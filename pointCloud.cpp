#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include "math/mathTypes.h"
#include <calib/calibration.h>
#include <calib/calibration.cpp>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <ros/ros.h>
#include <pcl/registration/icp.h>
#include<pcl/search/impl/kdtree.hpp>
#include <vector>

cv::Mat getSegmentedDepth(std::string depthFile, std::string maskFile){
    if(depthFile.empty()|maskFile.empty()){
        std::cerr<<"Error: No such depth image or mask \n";
    }
    cv::Mat depth, mask, depthMat;
    depth = cv::imread(depthFile, cv::IMREAD_ANYDEPTH);
    mask = cv::imread(maskFile, cv::IMREAD_GRAYSCALE);
    if(depth.empty()|mask.empty()){
        std::cerr<<"Error: Cannot open depth image or mask \n";
    }
    mask.convertTo(mask, CV_16UC1);
    depthMat = depth.mul(mask/255.0f);
    return depthMat;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr buildPointCloud(cv::Mat depthMat,  Calibration calibration, std::vector<hVec3D>* points){
    hVec2D uv ;
    hVec3D xyz;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud->width    = 480;
    cloud->height   = 640;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);

    for(int i = 0; i< depthMat.rows; ++i){
        for( int j = 0; j< depthMat.cols; ++j){   
            if(depthMat.at<ushort>(i,j)!=0){
                uv<<j,i,1;
                xyz = calibration.Unproject(uv);
                xyz *= 1.0f/xyz(2)*depthMat.at<ushort>(i,j);
                xyz(3) = 1;
                (*points).push_back(xyz);
                (*cloud)[depthMat.rows*j + i].x=xyz(0);
                (*cloud)[depthMat.rows*j + i].y=xyz(1);
                (*cloud)[depthMat.rows*j + i].z=xyz(2);
                (*cloud)[depthMat.rows*j + i].r=255;
                (*cloud)[depthMat.rows*j + i].g=255;
                (*cloud)[depthMat.rows*j + i].b=255;
            }
        }
    }
    return cloud;
}

int main(){

    transMatrix2D K; //camera instrinsics
    transMatrix3D L;

    K(0,0) = 525.0f; //fx
    K(0,2) = 319.5f; //x0
    K(1,1) = 525.0f; //fy
    K(1,2) = 239.5f; //y0
    K(2,2) = 1.0f;

    L<<1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1;

    Calibration calibration(K, L);

    cv::Mat depthMat = getSegmentedDepth("coffee.png", "mask.png");
    cv::Mat depthMat2 = getSegmentedDepth("coffee2.png", "mask2.png");
    cv::Mat depthMat3 = getSegmentedDepth("coffee3.png", "mask3.png");

    std::vector<hVec3D> points;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =  buildPointCloud(depthMat, calibration, &points);
   // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2 =  buildPointCloud(depthMat2, calibration);
   // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3 =  buildPointCloud(depthMat3, calibration);

    std::cout<<points.size()<<"\n";


//     pcl::visualization::CloudViewer viewer ("View Point Cloud");
//     //viewer.showCloud (cloud);
//    // viewer.showCloud (cloud2);
//     viewer.showCloud (cloud3);
//     while (!viewer.wasStopped ()){}

    return 0;
}