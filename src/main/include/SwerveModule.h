// Copyright (c) 2023 FRC Team 4918.
// Some software is also Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <numbers>

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

#include <rev/SparkMax.h>
#include <frc/AnalogInput.h>
#include <ctre/Phoenix.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

class SwerveModule
{
public:
   /*
   SwerveModule(int driveMotorChannel, int turningMotorChannel,
                int driveEncoderChannelA, int driveEncoderChannelB,
                int turningEncoderChannelA, int turningEncoderChannelB);
     */

   SwerveModule(int driveMotorCanID,
                int turningMotorCanID,
                int turningEncoderCanID,
                int turningEncoderOffset);

   frc::SwerveModulePosition GetPosition();
   void SetDesiredState( const frc::SwerveModuleState &state, bool bFreezeDriveMotor = false );

private:
   static constexpr double kWheelRadius = 0.319;
   // CHANGE THIS LATER ^^^^
   static constexpr int kEncoderResolution = 4096;

   static constexpr auto kModuleMaxAngularVelocity = std::numbers::pi * 10_rad_per_s; // radians per second
   static constexpr auto kModuleMaxAngularAcceleration = std::numbers::pi * 20_rad_per_s / 1_s; // radians per second^2

   ctre::phoenix6::hardware::TalonFX m_driveMotor;
   rev::spark::SparkMax m_turningMotor;

   ctre::phoenix6::hardware::CANcoder m_turningEncoder;

   //MM 2/23/25: Old swerve setup, remove once new swerve is working
   //rev::spark::SparkRelativeEncoder m_driveEncoder;
   //frc::AnalogInput m_turningEncoder;
   int m_turningEncoderOffset;

   frc::PIDController m_drivePIDController{
       0.1, //previously 1.0 on old swerve
       0.0,
       0.0};

   frc::ProfiledPIDController<units::radians> m_turningPIDController{
       // 8.0,  //[Something] too violent, 5 too slow
       // 0.0,
       // 0.001,
       // Values above are good for all 4 swerve module turn motors
       1.0, //previously 8.0 on old swerve
       0.0,
       0.0, //previously .001 on old swerve
       {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};
                                                                    // was 1.0
   frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{0.1_V, 1_V / 1_mps};
   frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{0.1_V /*was 0.1*/, 0.025_V /* was 0.025*/ / 1_rad_per_s};
};
// originally .5_V on line 72 made 0.025_V and 3_V on line 70 made 1_V
