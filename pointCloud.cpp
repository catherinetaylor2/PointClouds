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
#include <pcl/conversions.h>
#include <vector>

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

    return 0;
}