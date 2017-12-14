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
#include <pcl/search/impl/kdtree.hpp>
#include <vector>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;


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

PointCloud::Ptr buildPointCloud(cv::Mat depthMat, Calibration calibration, std::vector<hVec3D>* points){
    hVec2D uv ;
    hVec3D xyz;
    int numberOfPoints = 0;
    for(int i=0; i<depthMat.rows; ++i){
        for(int j=0; j<depthMat.cols; ++j){
            if(depthMat.at<ushort>(i,j)!=0){
                numberOfPoints++;
                uv<<j,i,1;
                xyz = calibration.Unproject(uv);
                xyz *= 1.0f/xyz(2)*depthMat.at<ushort>(i,j);
                (*points).push_back(xyz);
            }
        }
    }
    
    PointCloud::Ptr cloud (new PointCloud);
    cloud->is_dense = false;
    cloud->points.resize (numberOfPoints);

    for(int i=0; i<cloud->size(); ++i){
        (*cloud)[i].x = (*points)[i](0);
        (*cloud)[i].y = (*points)[i](1);
        (*cloud)[i].z = (*points)[i](2);
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
    std::vector<hVec3D> points, points2;

    cv::Mat depthMat = getSegmentedDepth("coffee/coffee.png", "coffee/mask.png");
    cv::Mat depthMat2 = getSegmentedDepth("coffee/coffee2.png", "coffee/mask2.png");
    PointCloud::Ptr cloud =  buildPointCloud(depthMat, calibration, &points);
    PointCloud::Ptr cloud2 =  buildPointCloud(depthMat2, calibration, &points2);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud);
    icp.setInputTarget(cloud2);

    PointCloud::Ptr Output (new PointCloud);
    icp.align(*Output);

    pcl::visualization::CloudViewer viewer ("View Point Cloud");
    viewer.showCloud (Output);
    while (!viewer.wasStopped ()){}

    return 0;
}