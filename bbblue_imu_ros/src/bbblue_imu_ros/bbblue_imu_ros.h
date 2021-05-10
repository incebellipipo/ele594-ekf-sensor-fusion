#ifndef BBBLUE_IMU_ROS_H_
#define BBBLUE_IMU_ROS_H_

#include "rc/mpu.h"
#include "rc/time.h"
#include "ros/ros.h"
#include "bbblue_imu_msgs/ImuRaw.h"
#include "sensor_msgs/Imu.h"

// bus for Robotics Cape and BeagleboneBlue is 2
// change this for your platform
#define I2C_BUS 2


class BlueImu {
public:
    BlueImu();

    void initialize();

private:
    rc_mpu_data_t data_;

    void readData();

    ros::NodeHandle nh_;

    ros::Publisher imu_raw_publisher_;

    ros::Publisher imu_publisher_;


};

#endif
