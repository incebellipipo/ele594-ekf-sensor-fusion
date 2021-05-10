#include "bbblue_imu_ros/bbblue_imu_ros.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "bbblue_ros_imu");

    BlueImu b;
    b.initialize();

    ros::spin();

    return 0;
}