#include "kungfury_filter.h"
#include "cmath"
#include "Eigen/Dense"

KungfuryFilter::KungfuryFilter() : nh_("~") {
    
    measurement_publisher_ = nh_.advertise<sensor_msgs::Imu>("/kungfury/measurement", 1000);
    predicted_model_publisher_ = nh_.advertise<sensor_msgs::Imu>("/kungfury/predicted_model", 1000);
    estimated_model_publisher_ = nh_.advertise<sensor_msgs::Imu>("/kungfury/estimated_model", 1000);
    raw_subscriber_ = nh_.subscribe("/imu_node/imu_raw",1000, &KungfuryFilter::rawCallback, this);
    
    initialize();
}

void KungfuryFilter::initialize() {
    z_ = Eigen::Vector3f::Zero();
    x_ = Eigen::Vector3f::Zero();
    x_prev_ = Eigen::Vector3f::Zero();
    i_ = Eigen::Vector3f::Zero();
    c_ = Eigen::Matrix3f::Zero();
    c_prev_ = Eigen::Matrix3f::Zero();
    
    r_ << 1, .3, .1,
            .3, 1, .3,
            .1, .3, 1;
    
    r_ = r_ * 1.5;
    q_ << 1, .5, .1,
            .5, 1, .5,
            .1, .5, 1;
    q_ = q_ * 2;
    h_ = Eigen::Matrix3f::Identity();
}

void KungfuryFilter::measure() {
    
    z_(0) = atan2(-raw_.accel.y, -raw_.accel.z);
    z_(1) = atan2(raw_.accel.x, sqrt(pow(raw_.accel.y, 2) + pow(raw_.accel.z, 2)));
    
    Eigen::Matrix3f R;

    R(0, 0) = cos(x_(1)) * cos(x_(2));
    R(0, 1) = sin(x_(0)) * sin(x_(1)) * cos(x_(2)) - cos(x_(0)) * sin(x_(2));
    R(0, 2) = cos(x_(0)) * sin(x_(1)) * cos(x_(2)) - sin(x_(0)) * sin(x_(2));

    R(1, 0) = cos(x_(1)) * sin(x_(2));
    R(1, 1) = sin(x_(0)) * sin(x_(1)) * sin(x_(2)) - cos(x_(1)) * cos(x_(2));
    R(1, 2) = cos(x_(0)) * sin(x_(1)) * sin(x_(2)) - sin(x_(0)) * cos(x_(2));

    R(2, 0) = -sin(x_(1));
    R(2, 1) = sin(x_(0)) * cos(x_(1));
    R(2, 2) = cos(x_(0)) * cos(x_(1));

    Eigen::Vector3f mag ;
    mag << raw_.mag.x, raw_.mag.y, raw_.mag.z;
    auto L_mag = R * mag;
    
    z_(2) = atan2(-L_mag(1), L_mag(2)) - i_(2);
    // z_(2) = atan2(raw_.mag.y, raw_.mag.x) - i_(2);
    
    auto quat_z = toQuaternion(z_);
   
    sensor_msgs::Imu msg;
    msg.header.frame_id = "measurement";
    msg.header.stamp = ros::Time::now();
    msg.orientation.w = quat_z.w();
    msg.orientation.x = quat_z.x();
    msg.orientation.y = quat_z.y();
    msg.orientation.z = quat_z.z();
    
    measurement_publisher_.publish(msg);

    if(first_measurement_) {
        x_prev_ = z_;
    }
}

void KungfuryFilter::predictState() {
    ros::Duration dT = raw_.header.stamp - raw_prev_.header.stamp;
    x_(0) = x_prev_(0) + dT.toSec() * (raw_.accel.x + raw_.accel.y * sin(x_prev_(0)) * tan(x_prev_(1)) + raw_.accel.z * cos(x_prev_(0)) * tan(x_prev_(1)));
    x_(1) = x_prev_(1) + dT.toSec() * (raw_.accel.y * cos(x_prev_(0)) - raw_.accel.z * sin(x_prev_(0)));
    x_(2) = x_prev_(2) * dT.toSec() * (raw_.accel.y * sin(x_prev_(0)) * ( 1 / cos(x_prev_(1))) + raw_.accel.z * cos(x_prev_(0)) * (1 / cos(x_prev_(1))));
   
    auto ra = raw_;
    ra.accel.x = 2 * ra.accel.x;
    ra.accel.y = 2 * ra.accel.y;
    ra.accel.z = 2 * ra.accel.z;
    xh_(0) = x_prev_(0) + dT.toSec() * (ra.accel.x + ra.accel.y * sin(x_prev_(0)) * tan(x_prev_(1)) + ra.accel.z * cos(x_prev_(0)) * tan(x_prev_(1)));
    xh_(1) = x_prev_(1) + dT.toSec() * (ra.accel.y * cos(x_prev_(0)) - ra.accel.z * sin(x_prev_(0)));
    xh_(2) = x_prev_(2) * dT.toSec() * (ra.accel.y * sin(x_prev_(0)) * ( 1 / cos(x_prev_(1))) + ra.accel.z * cos(x_prev_(0)) * (1 / cos(x_prev_(1))));
    
    auto quat_x = toQuaternion(x_);
    
    sensor_msgs::Imu msg;
    msg.header.frame_id = "predicted_model";
    msg.header.stamp = ros::Time::now();
    msg.orientation.w = quat_x.w();
    msg.orientation.x = quat_x.x();
    msg.orientation.y = quat_x.y();
    msg.orientation.z = quat_x.z();
    
    predicted_model_publisher_.publish(msg);
}

void KungfuryFilter::calculateProcessJacobian() {
    ros::Duration dT = raw_.header.stamp - raw_prev_.header.stamp;
    g_(0, 0) = 1 + dT.toSec() * (tan(x_prev_(1) * (raw_.accel.y * cos(x_prev_(0)) - raw_.accel.z * sin(x_prev_(0)))));
    g_(0, 1) = dT.toSec() * ((1/pow(cos(x_prev_(1)),2)) * (raw_.accel.y * sin(x_prev_(0) + raw_.accel.z * (cos(x_prev_(0))))));
    g_(0, 2) = 0;
    
    g_(1, 0) = dT.toSec() * (-raw_.accel.y * sin(x_prev_(0)) - raw_.accel.z * cos(x_prev_(0)));
    g_(1, 1) = 1;
    g_(1, 2) = 0;
    
    g_(2, 0) = dT.toSec() * ((1/cos(x_prev_(1))) * (raw_.accel.y * cos(x_prev_(0))) - raw_.accel.z * sin(x_prev_(0)));
    g_(2, 1) = dT.toSec() * (tan(x_prev_(1)) * (1/cos(x_prev_(1))) * (raw_.accel.y * sin(x_prev_(0)) + raw_.accel.z * cos(x_prev_(0))));
    g_(2, 2) = 1;
}

void KungfuryFilter::predictCovariance() {
    calculateProcessJacobian();
    c_ = g_ * c_prev_ * g_.transpose() + r_;
}

void KungfuryFilter::calculateKalmanGain() {
    k_ = c_ * h_.transpose() * ( h_ * c_ * h_.transpose() + q_).inverse();
}

void KungfuryFilter::estimateState() {
    x_ = x_ + k_ * (z_ - xh_);
    auto quat_x = toQuaternion(x_);
    
    sensor_msgs::Imu msg;
    msg.header.frame_id = "estimated_model";
    msg.header.stamp = ros::Time::now();
    msg.orientation.w = quat_x.w();
    msg.orientation.x = quat_x.x();
    msg.orientation.y = quat_x.y();
    msg.orientation.z = quat_x.z();
    
    estimated_model_publisher_.publish(msg);
}

void KungfuryFilter::estimateCovariance() {
    c_ = (Eigen::Matrix3f::Identity() - k_ * h_) * c_;
}

void KungfuryFilter::rawCallback(const bbblue_imu_msgs::ImuRaw msg) {
    raw_ = msg;
    
    measure();
   
    if(first_measurement_) {
        first_measurement_ = false;
        raw_prev_ = raw_;
        return;
    }
    
    predictState();
 
    predictCovariance();
    
    calculateKalmanGain();
    
    estimateState();
    
    estimateCovariance();
    
    x_prev_ = x_;
    c_prev_ = c_;
    raw_prev_ = raw_;
}



Eigen::Quaternionf KungfuryFilter::toQuaternion(Eigen::Vector3f euler) {
    return Eigen::AngleAxisf(euler.x(), Eigen::Vector3f::UnitX())
         * Eigen::AngleAxisf(euler.y(), Eigen::Vector3f::UnitY())
         * Eigen::AngleAxisf(euler.z(), Eigen::Vector3f::UnitZ());
}
