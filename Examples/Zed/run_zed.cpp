#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
#include <iostream>
#include "System.h"

using namespace std;
using namespace sl;

cv::Mat slMat2cvMat(sl::Mat& input);

int main( int argc, char** argv )
{
    if (argc != 3)
    {
        cout<<"Usage: run_zed path_to_vocabulary path_to_config_file"<<endl;
        return 0;
    }
    sl::Camera zed;// Create a ZED camera object
    //Set configuration parameters
       InitParameters init_params;
       init_params.camera_resolution = RESOLUTION_VGA; // cost time: 0.04647
       //init_params.camera_resolution = RESOLUTION_HD720;//cost time: 0.086479
       init_params.camera_fps = 60; // Set fps at 60

       // Open the camera
       ERROR_CODE err = zed.open(init_params);
       if (err != SUCCESS){
           std::cout << sl::errorCode2str(err) << std::endl;
           zed.close();
           return 1; // Quit if an error occurred
       }
       RuntimeParameters init_grab;//设置grab()参数
       init_grab.enable_depth=true;
       init_grab.enable_point_cloud=false;

       int width = 672;
       int height = 376;
       cout<<"image width="<<width<<", height = "<<height<<endl;
       cv::Mat depth(height, width, CV_8UC4);
       cv::Mat color(height, width, CV_8UC4);
       ORB_SLAM2::System orbslam( argv[1], argv[2], ORB_SLAM2::System::RGBD, true );
       double index = 0;
       while (1)
       {
         if(zed.grab(init_grab)==SUCCESS)
                   {
                       sl::Mat image_depth,image_color;
                       zed.retrieveImage(image_color, VIEW_LEFT);
                       zed.retrieveImage(image_depth, VIEW_DEPTH);
                       cvtColor(slMat2cvMat(image_color),color,CV_BGRA2BGR);//转换成３通道图
                       cvtColor(slMat2cvMat(image_depth),depth,CV_BGRA2BGR);//转换成３通道图
                       orbslam.TrackRGBD( color, depth, index );

                   }
         if (cv::waitKey(5) == 'q')
         {
             break;
         }

         index += 0.1;

        }

    orbslam.Shutdown();
    zed.close();
    return 0;
}
cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}

