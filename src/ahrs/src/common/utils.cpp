#include "common/utils.h"

void toEulerAngle(const Eigen::Quaterniond& q, double& roll, double& pitch, double& yaw)
{
    // roll (x-axis rotation)
    double sinr = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
    yaw = atan2(siny, cosy);
}

Eigen::Quaterniond toQuaternion(double pitch, double roll, double yaw)
{
    Eigen::Quaterniond q;
        // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    q.w() = cy * cr * cp + sy * sr * sp;
    q.x() = cy * sr * cp - sy * cr * sp;
    q.y() = cy * cr * sp + sy * sr * cp;
    q.z() = sy * cr * cp - cy * sr * sp;
    return q;
}

Eigen::Matrix3d vX(Eigen::Vector3d v){
    Eigen::Matrix3d re;
    re<<0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return re;
}

Eigen::Matrix3d regularMat(Eigen::Matrix3d M){
    //Eigen::Matrix<double, 1,3> x=M.row(1);
    //Eigen::Matrix<double, 1,3> y=M.row(2);
    Eigen::Vector3d x=M.row(1);
    Eigen::Vector3d y=M.row(2);
    double err = x.dot(y);
    Eigen::Vector3d xx = x - err/2*y;
    Eigen::Vector3d yy = y - err/2*x;
    Eigen::Vector3d zz = xx.cross(yy);
    Eigen::Matrix3d re;
    re.block(0,0,1,3)=.5*(3-xx.dot(xx))*xx;
    re.block(1,0,1,3)=.5*(3-yy.dot(yy))*yy;
    re.block(2,0,1,3)=.5*(3-zz.dot(zz))*zz;
    return re;
}

Eigen::Matrix3d ang2Rbn(double psi,double theta,double phi){
    double sp = sin(psi);
    double cp = cos(psi);
    double st = sin(theta);
    double ct =cos(theta);
    double sph = sin(phi);
    double cph = cos(phi);
    Eigen::Matrix3d R;
    R(0,0) = ct*cp;
    R(0,1) = sph*st*cp - cph*sp;
    R(0,2) = cph*st*cp + sph*sp;
    R(1,0) = ct*sp;
    R(1,1) = sph*st*sp + cph*cp;
    R(1,2) = cph*st*sp - sph*cp;
    R(2,0) = -st;
    R(2,1) = sp*ct;
    R(2,2) = cph*ct;
    return R;
}
