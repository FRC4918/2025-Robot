// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/Joystick.h>
#include "Drivetrain.h"
#include <iostream>

//Robot Pos Variables
static frc::Pose2d pose;

class Robot : public frc::TimedRobot {

//JOYSTICKint
frc::Joystick m_Console{3};

  public:
    void RobotInit() override {
        //initialize motors/sensors/etc. here
    }

    void AutonomousInit() override {
        //auto code goes here
    }

    void TeleopInit() override {
      //initialize teleop-specific motors/sensors/etc. here
    }

    void TeleopPeriodic() override {
  
      DriveWithJoystick(true);
      m_swerve.UpdateOdometry();
    }

  private:
    frc::XboxController m_driverController{0};
    frc::XboxController m_operatorController{1};
    Drivetrain m_swerve;
    

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    //MM (1/4/25): I don't trust this - could be unnecessarily limiting our motors.
    //             Could be good to test with and without to see how they impact
    //             drive and handling
    frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{10 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{10 / 1_s};
    frc::SlewRateLimiter<units::scalar> m_rotLimiter{   10 / 1_s};

    void DriveWithJoystick(bool fieldRelative) {
      // SLOOOOOOWMODE
      // low speed by 0.3 if holding r-trigger //0.2
      double lowGear = m_driverController.GetRightTriggerAxis() > 0.1 ? 0.3 : 1.0;

      // Get the x speed. We are inverting this because Xbox controllers return
      // negative values when we push forward.
      auto xSpeed = -m_xspeedLimiter.Calculate(
                              frc::ApplyDeadband(m_driverController.GetLeftY(), 0.2)) *
                          Drivetrain::kMaxSpeed * lowGear;

      // Get the y speed or sideways/strafe speed. We are inverting this because
      // we want a positive value when we pull to the left. Xbox controllers
      // return positive values when you pull to the right by default.
      auto ySpeed = -m_yspeedLimiter.Calculate(
                              frc::ApplyDeadband(m_driverController.GetLeftX(), 0.2)) *
                          Drivetrain::kMaxSpeed * lowGear;

      // Get the rate of angular rotation. We are inverting this because we want a
      // positive value when we pull to the left (remember, CCW is positi std::cout << "Wheel angles FL,FR/BL,BR:"
              
      // mathematics). Xbox controllers return positive values when you pull to
      // the right by default.
      auto rot = -m_rotLimiter.Calculate(
                          frc::ApplyDeadband(m_driverController.GetRightX(), 0.2)) *
                      Drivetrain::kMaxAngularSpeed * lowGear;

    // Relative Switch (A)
    fieldRelative = !m_driverController.GetAButton();

    // Reset Gyro and Position (Y)
    if (m_driverController.GetYButton()) {
      m_swerve.Reset();
    }

    if (m_driverController.GetXButton()) {
      xSpeed = Drivetrain::kMaxSpeed / 4;
    }
    
    // Brake (B) and drive
    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, m_driverController.GetBButton());

    } //End DriveWithJoystick

};

int main() {
  return frc::StartRobot<Robot>();
}