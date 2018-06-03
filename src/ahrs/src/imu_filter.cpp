#include "imu_filter.h"
#include <math.h>
#include <cstdio>
#include <stdio.h>
#include <iomanip>
#include <visualization_msgs/Marker.h>

imu_filter::imu_filter(){
    init=false;
    bgq<<4.3e-6, 4.6e-6, 2.6e-6;
    baq<<1.2e-6, 2.7e-6, 1.1e-6;
    ros::NodeHandle n;
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}

bool imu_filter::check_static(imu_data data){
    if(data.id<100){
        return true;
    }else{
        return false;
    }
}

void imu_filter::processIMU(imu_data data){
    Eigen::Vector3d acc;
    acc<<data.ax,data.ay,data.az;
    Eigen::Vector3d gyro;
    gyro<<data.gx,data.gy,data.gz;
    if(init==false){
        std::cout<<acc.transpose()<<std::endl;
        std::cout<<gyro.transpose()<<std::endl;
        Eigen::Vector3d t_acc = acc/acc.norm();
        double theta = asin(-t_acc(0));
        double phi = atan2(t_acc(1), t_acc(2));
        double psi = 0;
        Eigen::Matrix3d R = ang2Rbn(psi,theta,phi);
        std::cout<<R<<std::endl;
        status.R = R;
        last_ts=data.time;
        init=true;
        return;
    }
    double samT=data.time-last_ts;
    EKFPredict(acc, gyro, samT);
    if(check_static(data)){
        EKFUpdate();
    }
    publishPose();
}

void imu_filter::publishPose(){
    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = status.p(0);
    marker.pose.position.y = status.p(1);
    marker.pose.position.z = status.p(2);
    Eigen::Quaterniond q(status.R);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 2.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.5;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
}

void imu_filter::EKFPredict(Eigen::Vector3d acc,Eigen::Vector3d gyro,double samT){
    Eigen::Vector3d gravity_v;
    gravity_v<<0, 0, 9.8;
    Eigen::Matrix3d preR = (Eigen::Matrix3d::Identity()+samT*vX(gyro-status.bg))*status.R;
    status.R = regularMat(preR);
    Eigen::Vector3d prev = status.v + samT*( status.R*(acc-status.ba) - gravity_v );
    status.p = status.p + samT/2*(prev + status.v);
    status.ba = status.ba;
    status.bg = status.bg;
    status.v=prev;
    Eigen::Matrix<double, 15,15> Phi=Eigen::Matrix<double, 15,15>::Identity();
    Phi.block(0,12,3,3)=samT*preR;
    Phi.block(3,0,3,3)=samT*vX(preR*(acc-status.ba));
    Phi.block(3,9,3,3)=-samT*preR;
    Phi.block(6,3,3,3)=samT*Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 15,15> Q=Eigen::Matrix<double, 15,15>::Identity()*sqrt(samT);
    Eigen::Matrix3d Qg = sqrt(samT) * bgq.asDiagonal();
    Eigen::Matrix3d Qa = sqrt(samT) * baq.asDiagonal();
    Q.block(0,0,3,3)=Qg;
    Q.block(3,3,3,3)=Qa;
    Q.block(3,3,3,3)=Eigen::Matrix3d::Zero();
    status.cov = Phi*status.cov*Phi.transpose() + Q;
}    

void imu_filter::EKFUpdate(){
    Eigen::Vector3d err = -status.v;
    Eigen::Matrix<double, 3, 15> H;
    H.block(0,0,3,3)=Eigen::Matrix3d::Zero();
    H.block(3,0,3,3)=Eigen::Matrix3d::Identity();
    H.block(6,0,3,3)=Eigen::Matrix3d::Zero();
    H.block(9,0,3,3)=Eigen::Matrix3d::Zero();
    H.block(12,0,3,3)=Eigen::Matrix3d::Zero();
    Eigen::Matrix3d nRv = 1e-10*Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 15, 3> K = status.cov*H.transpose()*(H*status.cov*H.transpose() + nRv).inverse();
    Eigen::Matrix<double, 15, 1> dX = K*err;
    Eigen::Vector3d dR = dX.block(0,0,3,1);
    Eigen::Vector3d dv= dX.block(3,0,3,1);
    Eigen::Vector3d dp = dX.block(6,0,3,1);
    Eigen::Vector3d dba = dX.block(9,0,3,1);
    Eigen::Vector3d dbg = dX.block(12,0,3,1);
    status.cov = (Eigen::Matrix<double, 15,15>::Identity()-K*H)*status.cov;
    status.R = (Eigen::Matrix3d::Identity() - vX(dR))*status.R;
    status.v = status.v+dv;
    status.p = status.p+dp;
    status.ba = status.ba + dba;
    status.bg = status.bg + dbg;
}        