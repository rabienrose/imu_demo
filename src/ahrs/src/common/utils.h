#ifndef __UTILS_H_INCLUDED__ 
#define __UTILS_H_INCLUDED__ 

#include <Eigen/Dense>
#include <iostream>
#include <fstream>

struct imu_data{
    imu_data(){
        time=-1;
    }
    int id;
    double time;
    double ax,ay,az,gx,gy,gz;
};

void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw);
Eigen::Quaterniond toQuaternion(double pitch, double roll, double yaw);
Eigen::Vector4d fromtwovectors(Eigen::Vector3d u, Eigen::Vector3d v);
Eigen::Matrix3d ang2Rbn(double psi,double theta,double phi);
Eigen::Matrix3d regularMat(Eigen::Matrix3d M);
Eigen::Matrix3d vX(Eigen::Vector3d v);

#endif