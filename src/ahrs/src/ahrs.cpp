#include "ahrs.h"
#include <math.h>
#include <cstdio>
#include <stdio.h>
#include <iomanip>
#include <visualization_msgs/Marker.h>

AHRS::AHRS(){
    deltat = 1/(double)100; 
    noise_gyro = 2.4e-3; 
    noise_accel = 2.83e-2;  
    gravity = 9.8;  
    bias_w << -0.0055,    0.0324,    0.0140;
    bias_a << 0.0159,    0.0269,   0.0526;
    P= 1e-10*Eigen::Matrix<double, 4, 4>::Identity();
    x<<0,1,0,0;
    x0=x;
    m_fpImu = fopen("imu_re.txt", "w+");
    ros::NodeHandle n;
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
}

void AHRS::publishDir(Eigen::Vector4d q){
    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = q(1);
    marker.pose.orientation.y = q(2);
    marker.pose.orientation.z = q(3);
    marker.pose.orientation.w = q(0);
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

void AHRS::processIMU(imu_data data){
    if(last_imu.time<0){
        last_imu=data;
        Eigen::Vector3d a;
        a<<data.ax, data.ay, data.az;
        Eigen::Vector3d iden;
        iden<<0, 0, 1;
        Eigen::Quaterniond quad;
        quad.setFromTwoVectors(a, iden);
        x<<quad.w(), quad.x(), quad.y(), quad.z();
        //std::cout<<x<<std::endl;
        return;
    }
    Eigen::Vector3d w;
    w[0]=(last_imu.gx+data.gx)/2;
    w[1]=(last_imu.gy+data.gy)/2;
    w[2]=(last_imu.gz+data.gz)/2;
    w = w - bias_w;
    Eigen::Matrix<double, 4,4> Omega;
    Omega<< 0,    -w(0),  -w(1),  -w(2),
            w(0), 0,      w(2),   -w(1),
            w(1), -w(2),  0,       w(0),
            w(2), w(1),   -w(0),    0;
    
    Eigen::Matrix<double, 4,4> F = Eigen::Matrix<double, 4, 4>::Identity() + deltat * Omega / 2;
    //std::cout<<std::setprecision(15)<<F(0,3)<<std::endl;
    Eigen::Matrix<double, 4,3> G;
    G <<  -x(1),  -x(2), -x(3),
          x(0),   -x(3),  x(2),
          x(3),   x(0),  -x(1),
         -x(2),   x(1),   x(0);
    G=G/2;
    Eigen::Matrix<double, 4,4> Q = pow((noise_gyro * deltat),2) * (G * G.transpose());
    //fprintf(m_fpImu, "%f,%f,%f\n", Q(2,3)*1e12, Q(2,1)*1e12, Q(0,3)*1e12);
    x = F * x;
    x = x / x.norm(); 
    P = F * P * F.transpose() + Q;
    
    Eigen::Vector3d a;
    a<< data.ax, data.ay, data.az;
    a = a - bias_a;
    
    Eigen::Vector3d ea = a / a.norm();
    Eigen::Vector3d ea_prediction;
    ea_prediction <<  2*(x(1)*x(3)-x(0)*x(2)),
                     2*(x(2)*x(3)+x(0)*x(1)),
                     x(0)*x(0)-x(1)*x(1)-x(2)*x(2)+x(3)*x(3);
    
    Eigen::Vector3d y = ea - ea_prediction;
    //std::cout<<y(0)<<","<<y(1)<<","<<y(2)<<std::endl;
    
    Eigen::Matrix<double, 3,4> H;
    H  <<   -x(2),    x(3),    -x(0),   x(1),
            x(1),    x(0),     x(3),   x(2),
            x(0),   -x(1),    -x(2),   x(3);
    H=H*2;
    double norm_noise_accel_2=pow(noise_accel / a.norm(),2);
    Eigen::Matrix<double, 3, 3> R_internal = norm_noise_accel_2 * Eigen::Matrix<double, 3, 3>::Identity();
    double norm_gravity_2=pow(1-gravity/a.norm(),2);
    Eigen::Matrix<double, 3, 3> R_external = norm_gravity_2* Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, 3, 3> R = R_internal + R_external;
    
    Eigen::Matrix<double, 3, 3> S = H * P * H.transpose() + R;
    Eigen::Matrix<double, 4, 3> K = P * H.transpose() * S.inverse();
    x = x + K * y;
    P = (Eigen::Matrix<double, 4, 4>::Identity() - K * H) * P;
    
    x = x / x.norm();
    P = (P + P.transpose()) / 2;
    publishDir(x);
    //Eigen::Quaterniond q(x(0), x(1), x(2), x(3));
    //Eigen::Quaterniond q0(x0(0), x0(1), x0(2), x0(3));
    //q=q0.inverse()*q;
    //double roll, pitch, yaw;
    //toEulerAngle(q, roll, pitch, yaw);
    
    //fprintf(m_fpImu, "%f,%f,%f,%f\n", x(0), x(1), x(2), x(3));
    //fprintf(m_fpImu, "%f,%f,%f\n", roll, pitch, yaw);
    last_imu=data;
}