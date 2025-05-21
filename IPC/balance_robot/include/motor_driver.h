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
    // Stepper motor pins (matching your setup)
    static const int LEFT_STEP_PIN = 14;
    static const int LEFT_DIR_PIN = 4;
    static const int RIGHT_STEP_PIN = 17;
    static const int RIGHT_DIR_PIN = 16;
    static const int ENABLE_PIN = 15;
    
    // Stepper motor objects
    ContinuousStepper<StepperDriver> left_stepper_;
    ContinuousStepper<StepperDriver> right_stepper_;
    
    // Motor parameters
    static const int MAX_SPEED = 1400;  // Maximum acceleration from your code
};

#endif // MOTOR_DRIVER_H 