/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <thread>



namespace ORB_SLAM2
{

#define MATCHES_DIST_THRESHOLD 25

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
{}

/**
 * @brief Copy constructor
 *
 * 复制构造函数, mLastFrame = Frame(mCurrentFrame)
 */
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),

     //新添加的成员变量
     mvKeylines(frame.mvKeylines),mvKeylinesRight(frame.mvKeylinesRight),
     mLine1StartPoints(frame.mLine1StartPoints),mLine2StartPoints(frame.mLine2StartPoints),
     mLine1EndPoints(frame.mLine1EndPoints),mLine2EndPoints(frame.mLine2EndPoints),
     mvLineStartDepth(frame.mvLineStartDepth),mvLineEndDepth(frame.mvLineEndDepth),
     mLineStartPoints_3(frame.mLineStartPoints_3),mLineEndPoints_3(frame.mLineEndPoints_3),
     mlinedescriptors(frame.mlinedescriptors.clone()),mlinedescriptorsRight(frame.mlinedescriptorsRight.clone())
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}


// 双目的初始化
Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mb(0), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    // 同时对左右目提特征//线程
    //使用引用函数传递  前提是提供一个合适的对象指针作为第一个参数，这里用的是this指的是-->Frame类对象
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);//this的存在意义？章节2.2传递参数给线程函数
    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    thread threadLine(&Frame::ExtractLSD,this,imLeft,imRight);

    threadLeft.join();
    threadRight.join();
    threadLine.join();
   // std::cout<<"ok"<<std::endl;
   // ExtractLSD(imLeft,imRight);//this的存在意义？章节2.2传递参数给线程函数


    std::cout<<mvLineEndDepth.size()<<std::endl;

    ///注释1: Frame::ExtractORB.线程传参（int, cv::Mat）
/*void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}*/

    N = mvKeys.size();//原始左图像提取出的特征点（未校正）

    if(mvKeys.empty())
        return;
    /// Undistort特征点，这里没有对双目进行校正，因为要求输入的图像已经进行极线校正
    //这是UndistortKeyPoints()对畸变参数的检测，如果为0，就直接赋值。
    /*if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }*/
    UndistortKeyPoints();//得到矫正过后的特征点向量mvKeysUn

    // 计算双目间的匹配, 匹配成功的特征点会计算其深度
    // 深度存放在mvuRight 和 mvDepth 中
    ComputeStereoMatches();

    // 对应的mappoints
    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));   
    mvbOutlier = vector<bool>(N,false);


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)//初始化默认true
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);//和确定在那个格子有关
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();//分配到每个格子里
}

// RGBD初始化
Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();    
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);//提取orb

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();//矫正

    ComputeStereoFromRGBD(imDepth);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

// 单目初始化
Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    // 调用OpenCV的矫正函数矫正orb提取的特征点
    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);//这是一个系数，总的行数/这个总距离(0.05-0.07)
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);//(0.05-0.07)

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);//总数/格子数
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
        //std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
            mGrid[i][j].reserve(nReserve);

    // 在mGrid中记录各特征点
    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}
///frame extract lsd_line 提取左相机特征和描述子
/// match
/// 提取起始点和终点
/// 计算深度
int Frame::ExtractLSD(const cv::Mat &im1,const cv::Mat &im2)
{
    cv::Mat imageMat1 =im1.clone();
    cv::Mat imageMat2 =im2.clone();
      if( imageMat1.data == NULL || imageMat2.data == NULL )
      {
        std::cout << "Error, image could not be loaded. Please, check its path" << std::endl;

      }
      /* create binary masks */
        cv::Mat mask1 = Mat::ones( imageMat1.size(), CV_8UC1 );
        cv::Mat mask2 = Mat::ones( imageMat2.size(), CV_8UC1 );

        /* create a pointer to a BinaryDescriptor object with default parameters */
        Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor( );
        /* compute lines and descriptors */

        bd->detect( imageMat1, mvKeylines, mask1 );
        bd->detect( imageMat2, mvKeylinesRight, mask2 );

        /* compute descriptors */
        bd->compute( imageMat1, mvKeylines, mlinedescriptors);
        bd->compute( imageMat2, mvKeylinesRight, mlinedescriptorsRight);

        /* select keylines from first octave and their descriptors */

        cv::Mat left_lbd, right_lbd;
        for ( int i = 0; i < (int) mvKeylines.size(); i++ )
        {
            if( mvKeylines[i].octave == 0 )
                left_lbd.push_back( mlinedescriptors.row( i ) );
        }

        for ( int j = 0; j < (int) mvKeylinesRight.size(); j++ )
        {
            if( mvKeylinesRight[j].octave == 0 )
                right_lbd.push_back( mlinedescriptorsRight.row( j ) );
        }

        //开始匹配
        Ptr<BinaryDescriptorMatcher> bdm = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
        /* require match */
        std::vector<cv::DMatch> matches;
        bdm->match( left_lbd, right_lbd, matches );

        /* select best matches */
        std::vector<cv::DMatch> good_matches;
        for ( int i = 0; i < (int) matches.size(); i++ )
        {
          if( matches[i].distance < MATCHES_DIST_THRESHOLD )
            good_matches.push_back( matches[i] );
        }
        //Line_N=(int)good_matches.size();
        //匹配好的线特征
        for ( int i = 0; i < (int) good_matches.size(); i++ )
        {
            KeyLine kp1,kp2;
            kp1=mvKeylines[good_matches[i].queryIdx];
            kp2=mvKeylinesRight[good_matches[i].trainIdx];
            Point2f p1_start =kp1.getStartPoint();
            Point2f p1_end=kp1.getEndPoint();

            Point2f p2_start =kp2.getStartPoint();
            Point2f p2_end=kp2.getEndPoint();
            mLine1StartPoints.push_back(p1_start);
            mLine2StartPoints.push_back(p2_start);
            mLine1EndPoints.push_back(p1_end);
            mLine2EndPoints.push_back(p2_end);
        }
        /// depth 是在这里计算的
        /// depth=baseline*fx/disparity
//        mvLineStartDepth.resize((int)mLine1StartPoints.size(),0);
//        mvLineEndDepth.resize((int)mLine1EndPoints.size(),0);
//        mvLineStartDepth.reserve((int)mLine1StartPoints.size());
//        mvLineEndDepth.reserve((int)mLine1EndPoints.size());
        mLineStartPoints_3.reserve((int)mLine1StartPoints.size());
        mLineEndPoints_3.reserve((int)mLine1EndPoints.size());

        for(int i = 0; i < (int) mLine1StartPoints.size(); i++ )
        {
            float disparty1=mLine1StartPoints[i].x-mLine2StartPoints[i].x;
            float disparty2=mLine1EndPoints[i].x-mLine2EndPoints[i].x;
            if(disparty1<mbf/mb && disparty1>0 && disparty2<mbf/mb && disparty2>0)
            {
                mvLineStartDepth.push_back(mbf/disparty1);
                mvLineEndDepth.push_back(mbf/disparty2);
                cv::Mat tmp1 = (cv::Mat_<float>(3,1) << mLine1StartPoints[i].x,mLine1StartPoints[i].y,mbf/disparty1);
                cv::Mat tmp2 = (cv::Mat_<float>(3,1) << mLine1EndPoints[i].x,mLine1EndPoints[i].y,mbf/disparty2);
                mLineStartPoints_3.push_back(tmp1);
                mLineEndPoints_3.push_back(tmp2);
            }
        }
        sort(mvLineStartDepth.begin(),mvLineStartDepth.end()); // 根据所有匹配对的SAD偏差进行排序, 距离由小到大
        sort(mvLineEndDepth.begin(),mvLineEndDepth.end());

        const float median1 = mvLineStartDepth[mvLineStartDepth.size()/2];
        const float median2 = mvLineEndDepth[mvLineEndDepth.size()/2];
        const float thDist1 = 1.5f*median1; // 计算自适应距离, 大于此距离的匹配对将剔除
        const float thDist2 = 1.5f*median2; // 计算自适应距离, 大于此距离的匹配对将剔除
        for(int i = 0; i < (int)mLineStartPoints_3.size(); i++)
        {
            if(mLineStartPoints_3[i].at<float>(2)>thDist1/2||mLineEndPoints_3[i].at<float>(2)>thDist2/2)
            {

                mLineStartPoints_3.erase(mLineStartPoints_3.begin()+i);
                mLineEndPoints_3.erase(mLineEndPoints_3.begin()+i);
            }
        }


        return 0;
}


/**
 * @brief Set the camera pose.
 * 
 * 设置相机姿态，随后会调用 UpdatePoseMatrices() 来改变mRcw,mRwc等变量的值
 * @param Tcw Transformation from world to camera
 */
void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

/**
 * @brief Computes rotation, translation and camera center matrices from the camera pose.
 *
 * 根据Tcw计算mRcw、mtcw和mRwc、mOw
 */
void Frame::UpdatePoseMatrices()
{
    // [x_camera 1] = [R|t]*[x_world 1]，坐标为齐次形式
    // x_camera = R*x_world + t
    mRcw = mTcw.rowRange(0,3).colRange(0,3);//旋转矩阵
    mRwc = mRcw.t();//坐标系转置
    mtcw = mTcw.rowRange(0,3).col(3);//平移向量
    // mtcw, 即相机坐标系下相机坐标系到世界坐标系间的向量, 向量方向由相机坐标系指向世界坐标系
    // mOw, 即世界坐标系下世界坐标系到相机坐标系间的向量, 向量方向由世界坐标系指向相机坐标系
    mOw = -mRcw.t()*mtcw;
}

/**
 * @brief 判断一个地图点是否在视野内
 *
 * 计算了重投影坐标，观测方向夹角，预测在当前帧的尺度
 * @param  pMP             MapPoint
 * @param  viewingCosLimit 视角和平均视角的方向阈值
 * @return                 true if is in view
 * @see SearchLocalPoints()
 */
bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos(); //获取在世界坐标系下的坐标

    // 3D in camera coordinates
    // 3D点P在相机坐标系下的坐标
    const cv::Mat Pc = mRcw*P+mtcw; // 这里的Rt是经过初步的优化后的
    const float &PcX = Pc.at<float>(0);
    const float &PcY = Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    // V-D 1) 将MapPoint投影到当前帧, 并判断是否在图像内
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    // V-D 3) 计算MapPoint到相机中心的距离, 并判断是否在尺度变化的距离内
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    // 世界坐标系下，相机到3D点P的向量, 向量方向由相机指向3D点P
    const cv::Mat PO = P-mOw;//P->O
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

    // Check viewing angle
    // V-D 2) 计算当前视角和平均视角夹角的余弦值, 若小于cos(60), 即夹角大于60度则返回
    cv::Mat Pn = pMP->GetNormal();///返回地图点平均观测方向，|Pn|=1

    const float viewCos = PO.dot(Pn)/dist;//算的cos

    if(viewCos<viewingCosLimit)//viewingCosLimit=0.5
        return false;

    // Predict scale in the image
    // V-D 4) 根据深度预测尺度（对应特征点在一层）
    const int nPredictedLevel = pMP->PredictScale(dist,this);//返回尺度

    // Data used by the tracking
    // 标记该点将来要被投影
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz; //该3D点投影到双目右侧相机上的横坐标u-b/z
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel = nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

/**
 * @brief 找到在 以x,y为中心,边长为2r的方形内且在[minLevel, maxLevel]的特征点
 * @param x        图像坐标u
 * @param y        图像坐标v
 * @param r        边长
 * @param minLevel 最小尺度
 * @param maxLevel 最大尺度
 * @return         满足条件的特征点的序号
 */
vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));//图像上的特征点坐标（x,y）经过换算后确定在哪一个格子方圆为r，且是左边界。单位是格子数
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));//方形r的右边界
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);//mnMinX格子的边界
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}

/**
 * @brief Bag of Words Representation
 *
 * 计算词包mBowVec和mFeatVec，其中mFeatVec记录了属于第i个node（在第4层）的ni个描述子
 * @see CreateInitialMapMonocular() TrackReferenceKeyFrame() Relocalization()
 */
void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

// 调用OpenCV的矫正函数矫正orb提取的特征点
void Frame::UndistortKeyPoints()
{
    // 如果没有图像是矫正过的，没有失真
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    // N为提取的特征点数量，将N个特征点的坐标保存在N*2的mat中
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    // 调整mat的通道为2，矩阵的行列形状不变
    mat=mat.reshape(2);

//https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html?highlight=undistortpoints#undistortpoints
    cv::undistortPoints(mat,                  //必须是2通道的  输入点
        mat,//输出点
        mK,//相机内参
        mDistCoef,//畸变参数
        cv::Mat(),
        mK//新的相机内参矩阵
    ); // 用cv的函数进行失真校正
    mat=mat.reshape(1);
    /*
    @param src Observed point coordinates, 1xN or Nx1 2-channel (CV_32FC2 or CV_64FC2).
    @param dst Output ideal point coordinates after undistortion and reverse perspective
transformation
    @param P New camera matrix (3x3) or new projection matrix (3x4)
CV_EXPORTS_W void undistortPoints( InputArray src, OutputArray dst,
                                   InputArray cameraMatrix, InputArray distCoeffs,
                                   InputArray R = noArray(), InputArray P = noArray());
    */

    // Fill undistorted keypoint vector
    // 存储校正后的特征点
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        // 矫正前四个边界点：(0,0) (cols,0) (0,rows) (cols,rows)
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0;         //左上
		mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; //右上
		mat.at<float>(1,1)=0.0;
		mat.at<float>(2,0)=0.0;         //左下
		mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; //右下
		mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));//左上和左下横坐标最小的
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));//右上和右下横坐标最大的
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));//左上和右上纵坐标最小的
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));//左下和右下纵坐标最小的
    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

/**
 * @brief 双目匹配
 *
 * 为左图的每一个特征点在右图中找到匹配点 \n
 * 根据基线(有冗余范围)上描述子距离找到匹配, 再进行SAD精确定位 \n
 * 最后对所有SAD的值进行排序, 剔除SAD值较大的匹配对，然后利用抛物线拟合得到亚像素精度的匹配 \n
 * 匹配成功后会更新 mvuRight(ur) 和 mvDepth(Z)
 */
void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);//mvuRight存储了左目像素点在右目中的对应点的横坐标
    mvDepth = vector<float>(N,-1.0f);//存储双目深度

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;//???

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;//最低层图像的行数

    //Assign keypoints to row table
    // 步骤1：建立特征点搜索范围对应表，一个特征点在一个带状区域内搜索匹配特征点
    // 匹配搜索的时候，不仅仅是在一条横线上搜索，而是在一条横向搜索带上搜索,简而言之，原本每个特征点的纵坐标为1，这里把特征点体积放大，纵坐标占好几行
    // 例如左目图像某个特征点的纵坐标为20，那么在右侧图像上搜索时是在纵坐标为18到22这条带上搜索，搜索带宽度为正负2，搜索带的宽度和特征点所在金字塔层数有关
    // 简单来说，如果纵坐标是20，特征点在图像第20行，那么认为18 19 20 21 22行都有这个特征点
    // vRowIndices[18]、vRowIndices[19]、vRowIndices[20]、vRowIndices[21]、vRowIndices[22]都有这个特征点编号
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());//初始化vRowIndices，长度和最低层图像的行数相同。每行预留200

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);//先reserve()200个空间

    const int Nr = mvKeysRight.size();//右相机特征点

    for(int iR=0; iR<Nr; iR++)//iR是右图特征点的索引，特征点分布在不同的金字塔层上，通过比分选择最佳的层数？？？
    {
        // !!在这个函数中没有对双目进行校正，双目校正是在外层程序中实现的
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        // 计算匹配搜索的纵向宽度，尺度越大（层数越高，距离越近），搜索范围越大
        // 如果特征点在金字塔第一层，则搜索范围为:正负2
        // 尺度越大其位置不确定性越高，所以其搜索半径越大
        //int octave; //!< octave (pyramid layer) from which the keypoint has been extracted
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];//每层搜索范围（条形）,每层的搜索半径都不一样。
        const int maxr = ceil(kpY+r);//是求不小于给定实数的最小整数
        const int minr = floor(kpY-r);//下取整

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);//最后vRowIndices每行存在可能匹配特征点的索引
    }

    // Set limits for search
    //双目基线mb米单位;mbf基线像素单位
    const float minZ = mb;        // NOTE bug mb没有初始化，mb的赋值在构造函数中放在ComputeStereoMatches函数的后面
    const float minD = 0;        // 最小视差, 设置为0即可
    //640x480 :maxD大概320mm
    const float maxD = mbf/minZ;  // 最大视差, 对应最小深度 mbf/minZ = mbf/mb = mbf/(mbf/fx) = fx (wubo???)

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    // 步骤2：对左目相机每个特征点，通过描述子在右目带状搜索区域找到匹配点, 再通过SAD做亚像素匹配
    // 注意：这里是校正前的mvKeys，而不是校正后的mvKeysUn
    // KeyFrame::UnprojectStereo和Frame::UnprojectStereo函数中不一致
    // 这里是不是应该对校正后特征点求深度呢？(wubo???)
    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];//遍历左目特征点
        const int &levelL = kpL.octave;//左目层数
        const float &vL = kpL.pt.y;//当前特征点纵坐标
        const float &uL = kpL.pt.x;//当前特征点横坐标

        // 可能的匹配点
        const vector<size_t> &vCandidates = vRowIndices[vL];//初始化vCandidates，拷贝vRowIndices[vL]的值

        if(vCandidates.empty())//可能图像某一行上面没有特征点
            continue;
        //640x480 :maxD大概320mm
        const float minU = uL-maxD; // 在右目上最小匹配范围  uL：是左目的坐标，显然＞右目与之匹配的横坐标
        const float maxU = uL-minD; // 在右目上最大匹配范围
        //640x480 :maxD大概320mm，minU所以很显然太近的话容易＜0
        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;//100
        size_t bestIdxR = 0;

        // 每个特征点描述子占一行，建立一个指针指向iL特征点对应的描述子
        const cv::Mat &dL = mDescriptors.row(iL);//此时依旧遍历左目当前特征点

        // Compare descriptor to right keypoints
        // 步骤2.1：遍历右目所有可能的匹配点，找出最佳匹配点（描述子距离最小）
        for(size_t iC=0; iC<vCandidates.size(); iC++)//vCandidates相当于基线上所有可能的特征点索引号
        {
            const size_t iR = vCandidates[iC];//其实vCandidates里面存的是索引号
            const cv::KeyPoint &kpR = mvKeysRight[iR];//通过索引号找到右目相应特征点

            // 仅对相邻尺度的特征点进行匹配
            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;//差距太大

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)//如果在应该的范围之内
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);//比较左右描述子(dL,dR)  CountBitsSetParallel

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;//既然是最佳，那就用不着vector,只要记住索引号就行
                }
            }
        } /// 到目前为止，最好的匹配的匹配误差存在bestDist，匹配点位置存在bestIdxR中
          ///左眼的第一个特征点已经找到“最佳适配点”
          ///大循环还没有结束

        // Subpixel match by correlation
        // 步骤2.2：通过SAD匹配提高像素匹配修正量bestincR
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            /// kpL.pt.x对应金字塔最底层坐标，将最佳匹配的特征点对尺度变换到尺度对应层 (scaleduL, scaledvL) (scaleduR0, )
            const float uR0 = mvKeysRight[bestIdxR].pt.x;//右目金字塔最底层  横坐标
            const float scaleFactor = mvInvScaleFactors[kpL.octave];//当前层octave所对应的尺度因子ScaleFactor
            const float scaleduL = round(kpL.pt.x*scaleFactor);//变换尺度后，也就是对应层的坐标
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);//右目金字塔相应层  横坐标
            //在匹配到的特征点的金字塔层，sad
            // sliding window search
            ///左图滑动窗口IL
            const int w = 5; // 滑动窗口的大小11*11 注意该窗口取自resize后的图像
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) * cv::Mat::ones(IL.rows,IL.cols,CV_32F);//窗口中的每个元素减去正中心的那个元素，简单归一化，减小光照强度影响

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1); // 11

            // 滑动窗口的滑动范围为（-L, L）,提前判断滑动窗口滑动过程中是否会越界
            const float iniu = scaleduR0+L-w; //这个地方是否应该是scaleduR0-L-w (wubo???)
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;
        ///右图也要有一个滑动窗口IR
            for(int incR=-L; incR<=+L; incR++)
            {
                // 横向滑动窗口
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) * cv::Mat::ones(IR.rows,IR.cols,CV_32F);//窗口中的每个元素减去正中心的那个元素，简单归一化，减小光照强度影响
                float dist = cv::norm(IL,IR,cv::NORM_L1); // 一范数，计算差的绝对值
                if(dist<bestDist)
                {
                    bestDist =  dist;// SAD匹配目前最小匹配偏差
                    bestincR = incR; // SAD匹配目前最佳的修正量
                }

                vDists[L+incR] = dist; // 正常情况下，这里面的数据应该以抛物线形式变化
            }

            if(bestincR==-L || bestincR==L) // 整个滑动窗口过程中，SAD最小值不是以抛物线形式出现，SAD匹配失败，同时放弃求该特征点的深度
                continue;

            // Sub-pixel match (Parabola fitting)
            // 步骤2.3：做抛物线拟合找谷底得到亚像素匹配deltaR
            // (bestincR,dist) (bestincR-1,dist) (bestincR+1,dist)三个点拟合出抛物线
            // bestincR+deltaR就是抛物线谷底的位置，相对SAD匹配出的最小值bestincR的修正量为deltaR
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));//抛物线拟合

            // 抛物线拟合得到的修正量不能超过一个像素，否则放弃求该特征点的深度
            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            // 通过描述子匹配得到匹配点位置为scaleduR0
            // 通过SAD匹配找到修正量bestincR
            // 通过抛物线拟合找到亚像素修正量deltaR
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);//最佳右目匹配坐标

            // 这里是disparity，根据它算出depth
            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD) // 最后判断视差是否在范围内
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                /// depth 是在这里计算的
                /// depth=baseline*fx/disparity
                mvDepth[iL]=mbf/disparity;   // 深度
                mvuRight[iL] = bestuR;       // 匹配对在右图的横坐标
                vDistIdx.push_back(pair<int,int>(bestDist,iL)); // 该特征点SAD匹配最小匹配偏差
            }
        }
    }

    // 步骤3：剔除SAD匹配偏差较大的匹配特征点
    // 前面SAD匹配只判断滑动窗口中是否有局部最小值，这里通过对比剔除SAD匹配偏差比较大的特征点的深度
    sort(vDistIdx.begin(),vDistIdx.end()); // 根据所有匹配对的SAD偏差进行排序, 距离由小到大
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median; // 计算自适应距离, 大于此距离的匹配对将剔除

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}


void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    // mvDepth直接由depth图像读取
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;//算出在虚拟右图中的坐标。
        }
    }
}

/**
 * @brief Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
 * @param  i 第i个keypoint
 * @return   3D点（相对于世界坐标系）
 */
cv::Mat Frame::UnprojectStereo(const int &i)
{
    // KeyFrame::UnprojectStereo
    // mvDepth是在ComputeStereoMatches函数中求取的
    // mvDepth对应的校正前的特征点，可这里却是对校正后特征点反投影
    // KeyFrame::UnprojectStereo中是对校正前的特征点mvKeys反投影
    // 在ComputeStereoMatches函数中应该对校正后的特征点求深度？？ (wubo???)
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

cv::Mat Frame::UnprojectStereoLineStart(const int &i)
{

    const float u = mLineStartPoints_3[i].at<float>(0);
    const float v = mLineStartPoints_3[i].at<float>(1);
    const float z = mLineStartPoints_3[i].at<float>(2);
    if(z>0)
    {
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

cv::Mat Frame::UnprojectStereoLineEnd(const int &i)
{

    const float u = mLineEndPoints_3[i].at<float>(0);
    const float v = mLineEndPoints_3[i].at<float>(1);
    const float z = mLineEndPoints_3[i].at<float>(2);
    if(z>0)
    {
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}


} //namespace ORB_SLAM
