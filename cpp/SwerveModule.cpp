// Copyright (c) 2023 FRC Team 4918.
// Some software is also Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "SwerveModule.h"

#include <numbers>
#include <iostream>

#include <frc/geometry/Rotation2d.h>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
// #include <rev/CANSparkMax.h>
// #include <rev/SparkMaxConfig.h>


using namespace rev::spark;
// #define SAFETY_LIMITS 1
// #undef SAFETY_LIMITS

/*---------------------------------------------------------------------*/
/* MotorInitSpark()                                                    */
/* Setup the initial configuration of a Neo motor, driven by a         */
/* Spark Max controller.  These settings can be superseded after this  */
/* function is called, for the needs of each specific SparkMax-driven  */
/* motor.                                                              */
/*---------------------------------------------------------------------*/
void MotorInitSpark(SparkMax &m_motor)
{
   SparkMaxConfig config{};

   // Set argument to true to also burn defaults into SparkMax flash.
   // 2025- not anymore
   //m_motor.RestoreFactoryDefaults();

   //m_motor.EnableSoftLimit(SparkMax::SoftLimitDirection::kForward,
                           //false);
   config.softLimit.ForwardSoftLimitEnabled(false);
   config.softLimit.ReverseSoftLimitEnabled(false);
   //m_motor.EnableSoftLimit(rev::spark::SparkMax::SoftLimitDirection::kReverse,
                           //false);
   /*
    * Configure Spark Max Output direction.
    * Sensor (encoder) direction would be set by
    * m_motorEncoder.SetInverted(true), but that doesn't work with
    * hall-effect encoders like we
    * have on our Neo drive motors.
    */
   // m_motor.SetInverted( true );  // invert direction of motor itself.
   //m_motor.SetInverted(false); // set forward direction of motor.
   config.Inverted(false);
   
   /* Set relevant frame periods to be at least as fast */
   /* as the periodic rate.                             */
   // See
   // ~/.gradle/caches/transforms-3/*/transformed/
   //   REVLib-cpp-2023.*-headers/rev/CANSparkMaxLowLevel.h
   //   for the default rates.
   // m_motor.SetPeriodicFramePeriod(
   //                     rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0,
   //                     10 );   // default 10?
   // m_motor.SetPeriodicFramePeriod(
   //                     rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1,
   //                     10 );   // default 20?
   // m_motor.SetPeriodicFramePeriod(
   //                     rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2,
   //                     20 );   // default 20?
   // m_motor.SetPeriodicFramePeriod(
   //                     rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus3,
   //                     20 );   // default 50?
   // These previous commands were like this for Talons:
   // m_motor.SetStatusFramePeriod(
   //                  StatusFrameEnhanced::Status_13_Base_PIDF0,  10, 10 );
   // m_motor.SetStatusFramePeriod(
   //                  StatusFrameEnhanced::Status_10_MotionMagic, 10, 10 );

   /* Set the peak and nominal outputs */
   //      m_motor.ConfigNominalOutputForward( 0, 10 );
   //      m_motor.ConfigNominalOutputReverse( 0, 10 );
   //      m_motor.ConfigPeakOutputForward(    1, 10 );
   //      m_motor.ConfigPeakOutputReverse(   -1, 10 );

   /* Set limits to how much current will be sent through the motor */
#ifdef SAFETY_LIMITS
                       // 10 Amps below 5000 RPM, above 5000 RPM it ramps from
                       // 10 Amps down to  5 Amps at 5700 RPM
   config.SmartCurrentLimit(10, 5, 5000);
#else
                       // 80 Amps below 5000 RPM, above 5000 RPM it ramps from
                       // 80 Amps down to 10 Amps at 5700 RPM
                       // We may have to try different CurrentLimits here to
                       // eliminate drivetrain chattering.
   //m_motor.SetSmartCurrentLimit(80, 10, 5000);
   config.SmartCurrentLimit(80, 10, 5000);
#endif
   config.limitSwitch.ForwardLimitSwitchType(rev::spark::LimitSwitchConfig::kNormallyOpen)
                     .ForwardLimitSwitchEnabled(false);
   // m_motor.GetForwardLimitSwitch(
   //            rev::SparkLimitSwitch::Type::kNormallyOpen)
   //     .EnableLimitSwitch(false);

   // Config 100% motor output to 12.0V
   config.VoltageCompensation(12.0);
  // m_motor.EnableVoltageCompensation(12.0);

   // Set ramp rate (how fast motor accelerates or decelerates).
   // We may have to try different RampRates here to
   // eliminate drivetrain chattering.
   //m_motor.SetClosedLoopRampRate(0.1); //0.1
   //m_motor.SetOpenLoopRampRate(0.1); //0.1
   config.OpenLoopRampRate(0.1);
   config.ClosedLoopRampRate(0.1);

   // m_motor.SetIdleMode( rev::CANSparkMax::IdleMode::kCoast );
   //m_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
   config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);

   m_motor.Configure(config, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);

} // MotorInitSpark()

SwerveModule::SwerveModule(const int driveMotorCanID,
                           const int turningMotorCanID,
                           const int turningEncoderSlot,
                           const int turningEncoderOffset)
  : m_driveMotor(driveMotorCanID, rev::spark::SparkMax::MotorType::kBrushless),
    m_turningMotor(turningMotorCanID, rev::spark::SparkMax::MotorType::kBrushless),
   m_driveEncoder(m_driveMotor.GetEncoder()),
   m_turningEncoder(turningEncoderSlot),
   m_turningEncoderOffset(turningEncoderOffset)
{
   
   MotorInitSpark(m_driveMotor);
   MotorInitSpark(m_turningMotor);


   // Initialize both CAN Spark-driven NEO motors.
   SparkMaxConfig drive_config{};
   SparkMaxConfig turn_config{};
   
   turn_config.SmartCurrentLimit(10, 5, 5000);
   drive_config.ClosedLoopRampRate(0.1);
   drive_config.OpenLoopRampRate(0.1);


   // m_turningMotor.SetSmartCurrentLimit(10, 5, 5000);
   // m_driveMotor.ClosedLoopRampRate(0.1);
   // m_driveMotor.OpenLoopRampRate(0.1);
   

   // Set the distance per pulse for the drive encoder. We can simply use the
   // distance traveled for one rotation of the wheel divided by the encoder
   // resolution.
   // m_driveEncoder.SetDistancePerPulse(2 * std::numbers::pi * kWheelRadius /
   //                                   kEncoderResolution);
   //m_driveEncoder.SetPositionConversionFactor(1.0 / 26.17 );
   drive_config.encoder.PositionConversionFactor(1.0 / 26.17);
   // We had a value of 1.0/39.0 above, and the robot then measured
   // 4.09 meters when it actually drove 20 feet (6.096 meters),
   // so adjust the new divisor: (39.0 * 4.09/6.096) = 26.17
   //m_driveEncoder.SetVelocityConversionFactor(1.0 / 2340.0);
   drive_config.encoder.VelocityConversionFactor(1.0 / 2340.0);

   //configure motors
   m_driveMotor.Configure(drive_config, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);
   m_turningMotor.Configure(turn_config, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);




   std::cout << "Initializing a swerve AnalogInput at "
                                            << turningEncoderSlot << std::endl;
   //  pm_turningEncoder = new frc::AnalogInput(turningEncoderSlot);
   //  pm_turningEncoder->GetAverageBits();
   //  pm_turningEncoder->GetOversampleBits();
   // std::cout << "receiving average/oversample bits "
   //           << m_turningMotor.GetAverageBits() << "/"
   //           << m_turningEncoder.GetOversampleBits() << std::endl;
   // std::cout << "receiving Sample Rate " << m_turningEncoder.GetSampleRate()
   //                                                                << std::endl;

   // Set the distance (in this case, angle) per pulse for the turning encoder.
   // This is the the angle through an entire rotation (2 * std::numbers::pi)
   // divided by the encoder resolution.
   // m_turningEncoder.SetDistancePerPulse(2 * std::numbers::pi /
   //                                     kEncoderResolution);

   // Limit the PID Controller's input range between -pi and pi and set the
   // input to be continuous.
   m_turningPIDController.EnableContinuousInput(
       -units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});

   // The following 2 calls save all settings permanently in the SparkMax's
   // flashes, so the settings survive even through a brownout.
   // These statements should be uncommented for at least one deploy-enable
   // Roborio cycle after any of the above settings change, but they should
   // be commented out between changes, to keep from using all the SparkMax's
   // flash-write cycles, because it has a limited number of flash-write
   // cycles.
   // m_driveMotor.BurnFlash();       // do this only when config changes.
   // m_turningMotor.BurnFlash();     // do this only when config changes.


}

// SwerveModule::SwerveModule(int driveMotorCanID,
//                            int turningMotorCanID,
//                            int turningEncoderAnaID):

// commented out 1/13 MM - doesn't appear to be called anywhere
//  frc::SwerveModuleState SwerveModule::GetState() const {
//    return {units::meters_per_second_t{m_driveEncoder.GetRate()},
//            units::radian_t{m_turningEncoder.GetVoltage()}};
//  }

frc::SwerveModulePosition SwerveModule::GetPosition() const
{
   
   return {units::meter_t{m_driveEncoder.GetPosition()},
           units::radian_t{(-((360 *
                               m_turningEncoder.GetAverageValue() / 4070 +
                               m_turningEncoderOffset) % 360)) *
                           std::numbers::pi / 180.0}};
}

void SwerveModule::SetDesiredState(
                                  const frc::SwerveModuleState &referenceState,
                                  bool bFreezeDriveMotor )
{
   // Optimize the reference state to avoid spinning further than 90 degrees
   const auto state = frc::SwerveModuleState::Optimize(
       referenceState, units::radian_t{
              (-((360 * m_turningEncoder.GetAverageValue() / 4070 +
                  m_turningEncoderOffset) % 360)) * std::numbers::pi / 180.0});
   // TODO: could this range actually be -pi to pi? currently 0 to 2pi,
   // docs just say it wants an angle

   // Calculate the drive output from the drive PID controller.

   const auto driveOutput = m_drivePIDController.Calculate(
       m_driveEncoder.GetVelocity(), state.speed.value());

   const auto driveFeedforward = m_driveFeedforward.Calculate(state.speed);

   // Calculate the turning motor output from the turning PID controller.
   const auto turnOutput = m_turningPIDController.Calculate(
       units::radian_t{ (-((360 * m_turningEncoder.GetAverageValue() / 4070 +
                                  m_turningEncoderOffset) % 360)) *
                        std::numbers::pi / 180.0 }, state.angle.Radians() );

   const auto turnFeedforward = m_turnFeedforward.Calculate(
       m_turningPIDController.GetSetpoint().velocity);

   /*std::cout << "drive rmp: "
             << m_driveEncoder.GetVelocity()
             << std::endl;*/
   // std::cout << " drive output " << driveOutput;
   // std::cout << " turn output " << turnOutput;
   // std::cout << " drive Feed Forward " << driveFeedforward.m_value;
   // std::cout << " turn FF " << turnFeedforward.value() << std::endl;
   //  Set the motor outputs.
   if ( bFreezeDriveMotor ) {
      m_driveMotor.SetVoltage(units::volt_t{0.0});
   } else {
      m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
   }
   m_turningMotor.SetVoltage(units::volt_t{turnOutput} - turnFeedforward);
}
