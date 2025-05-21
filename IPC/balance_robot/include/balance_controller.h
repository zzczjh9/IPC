#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>

class BalanceController {
public:
    BalanceController(ros::NodeHandle& nh);
    ~BalanceController() = default;

    // Initialize controller parameters
    void init();
    
    // Update control output using filtered IMU data
    void update(const double angle, const double angular_velocity);
    
    // Get motor commands
    void getMotorCommands(double& left_cmd, double& right_cmd);

private:
    // System parameters
    double mass_;           // Robot mass (kg)
    double length_;         // Distance from wheel axis to COM (m)
    double inertia_;        // Robot moment of inertia (kg*m^2)
    double wheel_radius_;   // Wheel radius (m)
    double max_tilt_;       // Maximum allowed tilt angle (rad)
    
    // State space matrices
    Eigen::Matrix4d A_;     // System matrix
    Eigen::Matrix<double, 4, 2> B_;  // Input matrix
    Eigen::Matrix4d Q_;     // State cost matrix
    Eigen::Matrix2d R_;     // Input cost matrix
    Eigen::Matrix<double, 2, 4> K_;  // LQR gain matrix
    
    // Current state and control
    Eigen::Vector4d x_;     // State vector [angle, angular_vel, position, velocity]
    Eigen::Vector2d u_;     // Control input [left_motor, right_motor]
    
    // Complementary filter parameters
    double alpha_ = 0.9;    // Filter coefficient from your code
    double last_angle_ = 0.0;
    double last_time_ = 0.0;
    
    // ROS members
    ros::NodeHandle& nh_;
    ros::Publisher motor_cmd_pub_;
    
    // Helper functions
    void calculateLQRGain();
    void updateSystemMatrices();
    void saturateControl();
};

#endif // BALANCE_CONTROLLER_H 