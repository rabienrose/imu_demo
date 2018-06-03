#ifndef __AHRS_H_INCLUDED__ 
#define __AHRS_H_INCLUDED__ 

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <common/utils.h>

class AHRS{
public:
    AHRS();
    void processIMU(imu_data data);
    void publishDir(Eigen::Vector4d q);
    double deltat; 
    double noise_gyro; 
    double noise_accel;  
    double gravity;  
    Eigen::Vector3d bias_w;
    Eigen::Vector3d bias_a;
    Eigen::Matrix<double, 4,4> P;
    Eigen::Vector4d x;
    Eigen::Vector4d x0;
    imu_data last_imu;
    FILE *m_fpImu;
    ros::Publisher marker_pub;
    
};

#endif