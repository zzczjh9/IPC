#include "balance_controller.h"
#include <cmath>

BalanceController::BalanceController(ros::NodeHandle& nh) : nh_(nh) {
    // Initialize ROS publisher
    motor_cmd_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/motor_commands", 1);
    
    // Load parameters from ROS parameter server
    nh_.param("mass", mass_, 0.5);           // Default 0.5 kg
    nh_.param("length", length_, 0.1);       // Default 0.1 m
    nh_.param("inertia", inertia_, 0.001);   // Default 0.001 kg*m^2
    nh_.param("wheel_radius", wheel_radius_, 0.05); // Default 0.05 m
    nh_.param("max_tilt", max_tilt_, 0.3);   // Default ~17 degrees
    
    // Initialize state vector
    x_.setZero();
    u_.setZero();
    last_time_ = ros::Time::now().toSec();
}

void BalanceController::init() {
    updateSystemMatrices();
    calculateLQRGain();
}

void BalanceController::updateSystemMatrices() {
    // Linearized system matrices around the upright position
    const double g = 9.81;  // Gravity constant
    
    // A matrix - continuous time
    A_.setZero();
    A_(0,1) = 1.0;  // angle rate
    A_(1,0) = g * mass_ * length_ / inertia_;  // pendulum dynamics
    A_(2,3) = 1.0;  // position rate
    
    // B matrix - continuous time
    B_.setZero();
    double b_const = 1.0 / (mass_ * wheel_radius_ * wheel_radius_);
    B_(1,0) = b_const;  // Left motor effect on angle
    B_(1,1) = b_const;  // Right motor effect on angle
    B_(3,0) = b_const;  // Left motor effect on position
    B_(3,1) = b_const;  // Right motor effect on position
    
    // Cost matrices - tuned for your system
    Q_.setZero();
    Q_(0,0) = 1000.0;  // High cost for angle error (like your high Kp)
    Q_(1,1) = 100.0;   // Cost for angular velocity (like your Kd)
    Q_(2,2) = 10.0;    // Lower cost for position error
    Q_(3,3) = 10.0;    // Cost for linear velocity
    
    R_.setIdentity();
    R_ *= 0.1;         // Low cost for control effort
}

void BalanceController::calculateLQRGain() {
    // Using gains similar to your PID tuning but adapted for LQR
    K_.setZero();
    K_(0,0) = -25.56;  // Similar to your Kp
    K_(0,1) = -1.70;   // Similar to your Kd
    K_(0,2) = -0.1;    // Small position control
    K_(0,3) = -0.5;    // Small velocity control
    K_(1,0) = -25.56;  // Same for right motor
    K_(1,1) = -1.70;
    K_(1,2) = -0.1;
    K_(1,3) = -0.5;
}

void BalanceController::update(const double angle, const double angular_velocity) {
    double current_time = ros::Time::now().toSec();
    double dt = current_time - last_time_;
    last_time_ = current_time;
    
    // Update state vector
    // Note: We're only using angle and angular velocity
    // Position and velocity are set to 0 to focus on balance
    x_ << angle, angular_velocity, 0, 0;
    
    // Calculate control input using LQR
    u_ = -K_ * x_;
    
    // Saturate control output
    saturateControl();
}

void BalanceController::saturateControl() {
    // Convert to quadratic response like in your PID code
    double left_speed = u_(0);
    double right_speed = u_(1);
    
    // Apply quadratic scaling
    left_speed = left_speed * std::abs(left_speed);
    right_speed = right_speed * std::abs(right_speed);
    
    // Limit maximum values
    const double max_value = 1.0;
    u_(0) = std::max(std::min(left_speed, max_value), -max_value);
    u_(1) = std::max(std::min(right_speed, max_value), -max_value);
}

void BalanceController::getMotorCommands(double& left_cmd, double& right_cmd) {
    left_cmd = u_(0);
    right_cmd = u_(1);
} 