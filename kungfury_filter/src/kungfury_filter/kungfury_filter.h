#ifndef KUNGFURY_FILTER_KUNGFURY_FILTER_H
#define KUNGFURY_FILTER_KUNGFURY_FILTER_H

#include "bbblue_imu_msgs/ImuRaw.h"
#include "ros/ros.h"
#include "Eigen/Dense"
#include "sensor_msgs/Imu.h"

typedef struct measurement_t {
    float roll;
    float pitch;
    float yaw;
} measurement_t;

class KungfuryFilter {
public:
    KungfuryFilter();

private:
   
    // Misc
    static Eigen::Quaternionf toQuaternion(Eigen::Vector3f euler);
    
    // Filter
    void initialize();
    Eigen::Vector3f i_;
    
    void measure();
    Eigen::Vector3f z_;
    Eigen::Vector3f z_prev_;
    bool first_measurement_ = true;
    
    void predictState();
    Eigen::Vector3f x_;
    Eigen::Vector3f xh_;
    Eigen::Vector3f x_prev_;
   
    void predictCovariance();
    Eigen::Matrix3f c_;
    Eigen::Matrix3f c_prev_;
   
    void calculateProcessJacobian();
    Eigen::Matrix3f g_;
    Eigen::Matrix3f r_;
    Eigen::Matrix3f q_;
   
    void calculateKalmanGain();
    Eigen::Matrix3f k_;
    Eigen::Matrix3f h_;
   
    void estimateState();
    
    void estimateCovariance();
   
    // ROS
    bbblue_imu_msgs::ImuRaw raw_;
    bbblue_imu_msgs::ImuRaw raw_prev_;
    
    ros::Subscriber raw_subscriber_;
    ros::NodeHandle nh_;
    void rawCallback(const bbblue_imu_msgs::ImuRaw msg);
    
    ros::Publisher measurement_publisher_;
    ros::Publisher predicted_model_publisher_;
    ros::Publisher estimated_model_publisher_;
};


#endif //KUNGFURY_FILTER_KUNGFURY_FILTER_H
