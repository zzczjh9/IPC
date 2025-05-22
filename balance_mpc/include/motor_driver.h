#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <ros/ros.h>
#include <ContinuousStepper.h>

class MotorDriver {
public:
    MotorDriver();
    ~MotorDriver();

    // Initialize stepper motors
    bool init();
    
    // Set motor speeds (-1.0 to 1.0)
    void setMotorSpeeds(double left_speed, double right_speed);

private:
    // Stepper motor pins (matching Orange Pi 5 Max pinout)
    // Left Motor
    static const int LEFT_STEP_PIN = GPIO3_B5;  // PWM13_M0 pin
    static const int LEFT_DIR_PIN = GPIO3_B2;   // PWM12_M0 pin
    
    // Right Motor
    static const int RIGHT_STEP_PIN = GPIO3_C2;  // PWM14_M0 pin
    static const int RIGHT_DIR_PIN = GPIO3_C1;   // PWM13_M2 pin
    
    // Enable pin
    static const int ENABLE_PIN = GPIO1_A7;      // PWM3_IR_M3 pin
    
    // I2C pins for MPU6050
    static const int I2C_SDA = GPIO1_B7;         // I2C2_SDA_M4 pin
    static const int I2C_SCL = GPIO1_A1;         // I2C2_SCL_M4 pin
    
    // Stepper motor objects
    ContinuousStepper<StepperDriver> left_stepper_;
    ContinuousStepper<StepperDriver> right_stepper_;
    
    // Motor parameters
    static const int MAX_SPEED = 1400;  // Maximum acceleration
};

#endif // MOTOR_DRIVER_H 