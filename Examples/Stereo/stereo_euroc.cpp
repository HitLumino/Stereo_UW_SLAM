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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<string>
#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

class Rectify
{
public:
    Rectify(const string& config);
    void doRectifyL(const cv::Mat& img_src, cv::Mat& img_rec);
    void doRectifyR(const cv::Mat& img_src, cv::Mat& img_rec);
    cv::Mat M1l,M2l,M1r,M2r;
};

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_EuRoC path_to_vocabulary path_to_settings path_to_sequence(/mav0)" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);

    const int nImages = vstrImageLeft.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    Rectify rectifyer(argv[2]);

    // Main loop
    cv::Mat imLeft, imRight;
    cv::Mat imLeftRec, imRightRec;
    ///
    ofstream f;
    f.open("Trajectory.txt");
    f << fixed;
    ///
    for(int ni=0; ni<nImages; ni++)
    {
        cout << "process image: " << ni <<endl;
        // Read left and right images from file
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);
        rectifyer.doRectifyL(imLeft, imLeftRec);
        rectifyer.doRectifyR(imRight, imRightRec);
        double tframe = vTimestamps[ni];

        if(imLeft.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(vstrImageLeft[ni]) << endl;
            return 1;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        cv::Mat Pose;
        // Pass the images to the SLAM system
        Pose=SLAM.TrackStereo(imLeftRec,imRightRec,tframe);


        cv::Mat Rwc = Pose.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Pose.rowRange(0,3).col(3);
    ///写入每一帧位姿
//        f << setprecision(9) <<
//             Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
//             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
//             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
        f << setprecision(9) <<twc.at<float>(0) << " "<<twc.at<float>(1) << " "<<twc.at<float>(2) << endl;
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            this_thread::sleep_for(std::chrono::microseconds((int)((T-ttrack)*1e6)));
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
}


Rectify::Rectify(const string& config)
{
    // Load settings related to stereo calibration
    cv::FileStorage fsSettings(config, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        //return -1;
        throw;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r,R,T;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    fsSettings["cam2to1.R"] >> R;
    fsSettings["cam2to1.T"] >> T;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];
    cv::Mat R1, R2, P1, P2, Q;
   // cv::Size imagesize =cv::Size(cols_l,rows_l);

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        //return -1;
        throw;
    }
//测试
    /*
    cv::stereoRectify(K_l,D_l,K_r,D_r,imagesize,R,T,R1,R2,P1,P2,Q,CV_CALIB_ZERO_DISPARITY,-1,cv::Size(cols_l,rows_l),0,0);
    cout<<"R1:   "<<R1<<endl;
    cout<<"R2:   "<<R2<<endl;
    cout<<"P1:   "<<P1<<endl;
    cout<<"P2:   "<<P2<<endl;
    */
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
}

void Rectify::doRectifyL(const cv::Mat& img_src, cv::Mat& img_rec)
{
    cv::remap(img_src,img_rec,M1l,M2l,cv::INTER_LINEAR);
}

void Rectify::doRectifyR(const cv::Mat& img_src, cv::Mat& img_rec)
{
    cv::remap(img_src,img_rec,M1r,M2r,cv::INTER_LINEAR);
}


void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/cam0/data.csv";
    string strPrefixLeft = strPathToSequence + "/cam0/data/";
    string strPrefixRight = strPathToSequence + "/cam1/data/";

    fTimes.open(strPathTimeFile.c_str());
    string s;
    getline(fTimes,s);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            int index = s.find_first_of(",");
            string t = s.substr(0,index);

            vTimestamps.push_back(stod(t)/10.0e8);
            vstrImageLeft.push_back(strPrefixLeft + t + ".png");
            vstrImageRight.push_back(strPrefixRight + t + ".png");
        }
    }
}
