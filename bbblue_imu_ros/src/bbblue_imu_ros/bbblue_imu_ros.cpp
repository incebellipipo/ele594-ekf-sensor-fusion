#include "bbblue_imu_ros.h"
#include "thread"
#include "functional"

BlueImu::BlueImu() : nh_("~") {


    imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("imu",10);
    imu_raw_publisher_ = nh_.advertise<bbblue_imu_msgs::ImuRaw>("imu_raw",10);

}

void BlueImu::initialize() {
    rc_mpu_config_t conf_ = rc_mpu_default_config();
    conf_.i2c_bus = I2C_BUS;
    conf_.enable_magnetometer = 1;
    conf_.show_warnings = 0;

    conf_.dmp_sample_rate = 40;
    // conf_.dmp_auto_calibrate_gyro = 1;

    // if(rc_mpu_initialize(&data_, conf_)) {}

    if(rc_mpu_initialize_dmp(&data_, conf_)) {
        fprintf(stderr,"rc_mpu_initialize_failed\n");
    }

    std::thread td(&BlueImu::readData, this);
    td.detach();

}

void BlueImu::readData() {

    sensor_msgs::Imu msg;
    bbblue_imu_msgs::ImuRaw msg_raw;


    while(ros::ok()) {

        rc_mpu_block_until_dmp_data();
	rc_usleep(10000);

        if(rc_mpu_read_accel(&data_) < 0) {std::cout << "problem accel" << std::endl;}
	
        msg.linear_acceleration.x = data_.accel[0];
        msg.linear_acceleration.y = data_.accel[1];
        msg.linear_acceleration.z = data_.accel[2];
        msg_raw.accel.x = data_.accel[0];
        msg_raw.accel.y = data_.accel[1];
        msg_raw.accel.z = data_.accel[2];

        if(rc_mpu_read_gyro(&data_) < 0) {std::cout << "problem gyro" << std::endl;}

	msg.angular_velocity.x = data_.gyro[0] * DEG_TO_RAD;
        msg.angular_velocity.y = data_.gyro[1] * DEG_TO_RAD;
        msg.angular_velocity.z = data_.gyro[2] * DEG_TO_RAD;
        msg_raw.gyro.x = data_.gyro[0] * DEG_TO_RAD;
        msg_raw.gyro.y = data_.gyro[1] * DEG_TO_RAD;
        msg_raw.gyro.z = data_.gyro[2] * DEG_TO_RAD;

        if(rc_mpu_read_mag(&data_) < 0) { std::cout << "problem mag" << std::endl;}

	msg_raw.mag.x = data_.mag[0];
        msg_raw.mag.y = data_.mag[1];
        msg_raw.mag.z = data_.mag[2];

        msg_raw.header.stamp = ros::Time::now();
        msg_raw.header.frame_id = "bbblue";

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "bbblue";
        msg.orientation.w = data_.fused_quat[0];
        msg.orientation.x = data_.fused_quat[1];
        msg.orientation.y = data_.fused_quat[2];
        msg.orientation.z = data_.fused_quat[3];


        imu_publisher_.publish(msg);
        imu_raw_publisher_.publish(msg_raw);
    }

    rc_mpu_power_off();
}
