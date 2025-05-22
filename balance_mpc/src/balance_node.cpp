#include <ros/ros.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "balance_controller.h"
#include "motor_driver.h"

// I2C pins for MPU6050 on Orange Pi 5 Max
#define I2C_SDA GPIO1_B7  // I2C2_SDA_M4 pin
#define I2C_SCL GPIO1_A1  // I2C2_SCL_M4 pin

class BalanceNode {
public:
    BalanceNode(ros::NodeHandle& nh) : controller_(nh) {
        // Initialize controller and motor driver
        controller_.init();
        if (!motor_driver_.init()) {
            ROS_ERROR("Failed to initialize motor driver");
            ros::shutdown();
            return;
        }
        
        // Initialize MPU6050
        setupMPU();
        
        // Initialize timer for control loop
        control_timer_ = nh.createTimer(ros::Duration(0.01), // 100Hz
                                      &BalanceNode::controlCallback, this);
                                      
        ROS_INFO("Balance node initialized");
    }

private:
    BalanceController controller_;
    MotorDriver motor_driver_;
    ros::Timer control_timer_;
    Adafruit_MPU6050 mpu_;
    
    // Sensor calibration values
    float base_x_accel_ = 0.0, base_y_accel_ = 0.0, base_z_accel_ = 0.0;
    float base_x_gyro_ = 0.0, base_y_gyro_ = 0.0, base_z_gyro_ = 0.0;
    
    // Complementary filter variables
    float last_angle_ = 0.0;
    float last_time_ = 0.0;
    const float alpha_ = 0.9;  // Filter coefficient
    
    void setupMPU() {
        // Initialize I2C with correct pins
        Wire.begin(I2C_SDA, I2C_SCL);
        if (!mpu_.begin()) {
            ROS_ERROR("Failed to find MPU6050 chip");
            ros::shutdown();
            return;
        }
        
        mpu_.setAccelerometerRange(MPU6050_RANGE_2_G);
        mpu_.setGyroRange(MPU6050_RANGE_250_DEG);
        mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);
        
        calibrateSensors();
    }
    
    void calibrateSensors() {
        ROS_INFO("Calibrating sensors...");
        const int num_readings = 2000;
        float x_accel = 0, y_accel = 0, z_accel = 0;
        float x_gyro = 0, y_gyro = 0, z_gyro = 0;
        
        sensors_event_t a, g, temp;
        
        for (int i = 0; i < num_readings; i++) {
            mpu_.getEvent(&a, &g, &temp);
            x_accel += a.acceleration.x / 16384;
            y_accel += a.acceleration.y / 16384;
            z_accel += a.acceleration.z / 16384;
            x_gyro += g.gyro.x / 131;
            y_gyro += g.gyro.y / 131;
            z_gyro += g.gyro.z / 131;
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
        sensors_event_t a, g, temp;
        mpu_.getEvent(&a, &g, &temp);
        
        // Calculate angles from accelerometer
        float accX = a.acceleration.x / 16384;
        float accY = a.acceleration.y / 16384;
        float accZ = a.acceleration.z / 16384;
        
        float angle_acc = atan2(-1 * accX, sqrt(pow(accY, 2) + pow(accZ, 2))) * 180 / M_PI - base_x_accel_;
        
        // Get angular velocity from gyro
        float gyro_rate = (g.gyro.y - base_y_gyro_) / 131;
        
        // Get current time
        float current_time = ros::Time::now().toSec();
        float dt = current_time - last_time_;
        last_time_ = current_time;
        
        // Complementary filter
        float angle = alpha_ * (last_angle_ + gyro_rate * dt) + (1.0 - alpha_) * angle_acc;
        last_angle_ = angle;
        
        // Update controller with filtered data
        controller_.update(angle * M_PI / 180.0, gyro_rate * M_PI / 180.0);
        
        // Get and apply motor commands
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