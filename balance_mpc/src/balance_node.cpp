#include <ros/ros.h>
#include "balance_controller.h"
#include "motor_driver.h"
#include "balance_mpc/mpu6050_linux.h"
#include <cmath>

// I2C pins for MPU6050 on Orange Pi 5 Max - These are now informational
// The actual bus is specified when creating MPU6050 object e.g. "/dev/i2c-2"
// #define I2C_SDA GPIO1_B7  // I2C2_SDA_M4 pin
// #define I2C_SCL GPIO1_A1  // I2C2_SCL_M4 pin

class BalanceNode {
public:
    BalanceNode(ros::NodeHandle& nh) : controller_(nh), mpu_() {
        // Initialize controller and motor driver
        controller_.init();
        if (!motor_driver_.init()) {
            ROS_ERROR("Failed to initialize motor driver");
            ros::shutdown();
            return;
        }
        
        // Initialize MPU6050
        if (!setupMPU()) {
            // Error already printed in setupMPU or mpu_.init()
            ros::shutdown();
            return;
        }
        
        // Initialize timer for control loop
        control_timer_ = nh.createTimer(ros::Duration(0.01), // 100Hz
                                      &BalanceNode::controlCallback, this);
                                      
        ROS_INFO("Balance node initialized");
    }

private:
    BalanceController controller_;
    MotorDriver motor_driver_;
    MPU6050 mpu_;
    ros::Timer control_timer_;
    
    // Sensor calibration values
    float base_x_accel_ = 0.0, base_y_accel_ = 0.0, base_z_accel_ = 0.0;
    float base_x_gyro_ = 0.0, base_y_gyro_ = 0.0, base_z_gyro_ = 0.0;
    
    // Complementary filter variables
    float last_angle_ = 0.0;
    float last_time_ = 0.0;
    const float alpha_ = 0.9;  // Filter coefficient
    
    bool setupMPU() {
        // Initialize I2C with correct pins - No longer done with Wire.h
        // Wire.begin(I2C_SDA, I2C_SCL);
        if (!mpu_.init()) {
            ROS_ERROR("Failed to initialize MPU6050 via mpu_linux.init()");
            return false;
        }
        
        calibrateSensors();
        return true;
    }
    
    void calibrateSensors() {
        ROS_INFO("Calibrating sensors...");
        const int num_readings = 2000;
        float x_accel = 0, y_accel = 0, z_accel = 0;
        float x_gyro = 0, y_gyro = 0, z_gyro = 0;
        
        MPU6050Data sensor_data;
        
        for (int i = 0; i < num_readings; i++) {
            if (!mpu_.getMotionData(sensor_data)) {
                ROS_WARN("Calibration: Failed to get MPU6050 data reading %d/%d", i+1, num_readings);
                ros::Duration(0.001).sleep();
                continue;
            }
            x_accel += sensor_data.accel_x;
            y_accel += sensor_data.accel_y;
            z_accel += sensor_data.accel_z;
            x_gyro += sensor_data.gyro_x;
            y_gyro += sensor_data.gyro_y;
            z_gyro += sensor_data.gyro_z;
            ros::Duration(0.001).sleep();
        }
        
        base_x_accel_ = x_accel / num_readings;
        base_y_accel_ = y_accel / num_readings;
        base_z_accel_ = z_accel / num_readings;
        base_x_gyro_ = x_gyro / num_readings;
        base_y_gyro_ = y_gyro / num_readings;
        base_z_gyro_ = z_gyro / num_readings;
        
        ROS_INFO("Calibration completed");
    }
    
    void controlCallback(const ros::TimerEvent&) {
        MPU6050Data sensor_data;
        if (!mpu_.getMotionData(sensor_data)) {
            ROS_ERROR("Control Loop: Failed to get MPU6050 data");
            motor_driver_.setMotorSpeeds(0,0);
            return;
        }
        
        float combined_yz_sq = pow(sensor_data.accel_y, 2) + pow(sensor_data.accel_z, 2);
        if (combined_yz_sq < 1e-6 && std::abs(sensor_data.accel_x) < 1e-6) {
            ROS_WARN_THROTTLE(1.0, "Accelerometer readings near zero, angle calculation might be unstable.");
        }

        float angle_acc = atan2(-1 * sensor_data.accel_x, sqrt(std::max(0.0f, combined_yz_sq))) * 180 / M_PI - base_x_accel_;
        
        float gyro_rate = sensor_data.gyro_y - base_y_gyro_;
        
        float current_time = ros::Time::now().toSec();
        float dt = current_time - last_time_;
        last_time_ = current_time;
        
        float angle = alpha_ * (last_angle_ + gyro_rate * dt) + (1.0 - alpha_) * angle_acc;
        last_angle_ = angle;
        
        controller_.update(angle * M_PI / 180.0, gyro_rate * M_PI / 180.0);
        
        double left_cmd, right_cmd;
        controller_.getMotorCommands(left_cmd, right_cmd);
        motor_driver_.setMotorSpeeds(left_cmd, right_cmd);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "balance_node");
    ros::NodeHandle nh;
    
    BalanceNode node(nh);
    
    ros::spin();
    
    return 0;
} 