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

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0)
{
}

/**
 * @brief Insert KeyFrame in the map
 * @param pKF KeyFrame
 */
void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);//锁住mMutexMap这个互斥元，如果已经被其他unique_lock锁住，暂时阻塞
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;//更新地图最新关键帧的id
}

//////////新添加的keyframes for局部BA地图2017.11.20
void Map::AddKeyFrameForBA(const std::vector<KeyFrame*> &pKFs)
{
    unique_lock<mutex> lock(mMutexMap);//锁住mMutexMap这个互斥元，如果已经被其他unique_lock锁住，暂时阻塞
    mvLocalBAKeyFrames.clear();//清空上一次缓存
    mvLocalBAKeyFrames=pKFs;
}
//////////新添加的keyframes for局部BA地图(参与但是不优化)2017.11.20
void Map::AddFixedKeyFrameForBA(const std::vector<KeyFrame*> &pKFs)
{
    unique_lock<mutex> lock(mMutexMap);//锁住mMutexMap这个互斥元，如果已经被其他unique_lock锁住，暂时阻塞
    mvLocalFixedFrames.clear();//清空上一次缓存
    mvLocalFixedFrames=pKFs;
}


/**
 * @brief Insert MapPoint in the map
 * @param pMP MapPoint
 */
void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);//锁住mMutexMap
    mspMapPoints.insert(pMP);
}


/**
 * @brief Erase MapPoint from the map
 * @param pMP MapPoint
 */
void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

/**
 * @brief Erase KeyFrame from the map
 * @param pKF KeyFrame
 */
void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

/**
 * @brief 设置参考MapPoints
 * @param vpMPs Local MapPoints
 */
void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}
///自己添加
void Map::SetReferenceLocalKFs(const vector<KeyFrame *> &vpKFs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvLocalKeyFrames = vpKFs;
}

void Map::SetParentKF(KeyFrame* pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    cur_ParentKF=pKF;
}

KeyFrame* Map::GetParentKFs()
{
    unique_lock<mutex> lock(mMutexMap);
    return cur_ParentKF;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}
///自己添加2017.11.20
vector<KeyFrame*> Map::GetLocalKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvLocalKeyFrames;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}


} //namespace ORB_SLAM
