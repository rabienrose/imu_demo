#ifndef __IMU_FILTER_H_INCLUDED__ 
#define __IMU_FILTER_H_INCLUDED__ 

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <common/utils.h>

class IMU_status{
public:
    IMU_status(){
        p<< 0,0,0;
        v<< 0,0,0;
        R=Eigen::Matrix<double, 3,3>::Identity();
        cov=Eigen::Matrix<double, 15,15>::Identity();
        ba<< 0,0,0;
        bg<< 0,0,0;
    }
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Matrix<double, 3,3> R;
    Eigen::Vector3d ba;
    Eigen::Vector3d bg;
    Eigen::Matrix<double, 15,15> cov;
};


class imu_filter{
public:
    imu_filter();
    void processIMU(imu_data data);
    void EKFUpdate();
    void EKFPredict(Eigen::Vector3d acc,Eigen::Vector3d gyro,double samT);
    void publishPose();
    bool check_static(imu_data data);
    IMU_status status;
    bool init;
    double last_ts;
    Eigen::Vector3d bgq;
    Eigen::Vector3d baq;
    ros::Publisher marker_pub;
};

#endif