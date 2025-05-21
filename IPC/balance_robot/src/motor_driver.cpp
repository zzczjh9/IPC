#include "motor_driver.h"
#include <cmath>

MotorDriver::MotorDriver() {
}

MotorDriver::~MotorDriver() {
    // Stop motors
    setMotorSpeeds(0, 0);
}

bool MotorDriver::init() {
    // Initialize stepper motors
    left_stepper_.begin(LEFT_STEP_PIN, LEFT_DIR_PIN);
    right_stepper_.begin(RIGHT_STEP_PIN, RIGHT_DIR_PIN);
    
    // Set acceleration limits
    left_stepper_.setAcceleration(MAX_SPEED);
    right_stepper_.setAcceleration(MAX_SPEED);
    
    // Enable motors
    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);  // Enable motors (active LOW)
    
    return true;
}

void MotorDriver::setMotorSpeeds(double left_speed, double right_speed) {
    // Convert normalized speeds (-1.0 to 1.0) to actual motor speeds
    // Using the same quadratic scaling as in your PID code
    left_speed = left_speed * std::abs(left_speed) * MAX_SPEED;
    right_speed = right_speed * std::abs(right_speed) * MAX_SPEED;
    
    // Update motor speeds
    left_stepper_.spin(left_speed);
    right_stepper_.spin(-right_speed);  // Negative for opposite direction
} 