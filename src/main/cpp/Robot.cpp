// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"
#include "MoreMotors.h"
#include "VisionStuff.h"

#include <cameraserver/CameraServer.h>
#include <frc/DriverStation.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/Joystick.h>
// #include <frc/smartdashboard/SmartDashboard.h>
#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/geometry/Transform3d.h>
#include <iostream>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/config/SparkMaxConfig.h>

#include <choreo/Choreo.h>



// #include <frc/Encoder.h>
// #include <frc/smartdashboard/SmartDashboard.h>

//Robot Pos Variables
//static units::angle::degree_t gyroYawHeading; //robot yaw (degrees)
static units::angular_velocity::degrees_per_second_t gyroYawRate; //robot rotate rate (degrees/second)
static frc::Pose2d pose;

//Camera Variables
//cs::UsbCamera camera1;
//cs::UsbCamera* cameras[] = {}
//static cs::CvSink cvSink;
//static int selectedTag = 1; // tag id


//PID idk
static double atPreviousError;
static double atIntegral;


// for auto and choreo stuff
frc::PIDController xController{20.0, 0.0, 0.0};//10//1
frc::PIDController yController{20.0, 0.0, 0.0};//10//1
frc::PIDController headingController{20.0, 0.0, 0.0};//7.5//1

frc::Timer timer;

//trajectories
auto traj_frfr = choreo::Choreo::LoadTrajectory<choreo::SwerveSample>("test1");


class Robot : public frc::TimedRobot {
  
  // Motors and robot vars

  // Elevator and sideshift
  rev::spark::SparkMax m_MasterElevatorMotor{ 9, rev::spark::SparkMax::MotorType::kBrushless };
  rev::spark::SparkMax m_SlaveElevatorMotor{ 10, rev::spark::SparkMax::MotorType::kBrushless };
  // rev::spark::SparkMax m_LeftSideshiftMotor{ 99999, rev::spark::SparkMax::MotorType::kBrushless };
  // rev::spark::SparkMax m_RightSideshiftMotor{ 99999, rev::spark::SparkMax::MotorType::kBrushless };
  SparkClosedLoopController m_ElevatorController = m_MasterElevatorMotor.GetClosedLoopController();


  //Coral motor
  WPI_VictorSPX m_CoralMotor{15};



//JOYSTICKint
frc::Joystick m_Console{3};

  public:
  // frc::Encoder leftEncoder{0, 1};  // DIO ports 0 and 1 for the left encoder ...
  // frc::Encoder rightEncoder{2, 3}; // DIO ports 2 and 3 for the right encoder ...


    void RobotInit() override {
      //initialize motors/sensors/etc. here
      //frc::SmartDashboard::PutString("My String", "Hello World!");
      //frc::SmartDashboard::PutBoolean("Holding Note", noteInShooter);
      
      //frc::SmartDashboard::PutString("Holding Note", noteInShooter);

      ElevatorMotorInitSpark(m_MasterElevatorMotor);
      ElevatorMotorInitSpark(m_SlaveElevatorMotor);
      
      // xController.SetTolerance(10.0);
      // yController.SetTolerance(10.0);
      // headingController.SetTolerance(10.0);

      rev::spark::SparkMaxConfig elevator_slave_config;
      elevator_slave_config.Follow(m_MasterElevatorMotor, true);
      m_SlaveElevatorMotor.Configure(elevator_slave_config, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);

      rev::spark::SparkMaxConfig elevator_master_config;
      elevator_master_config.closedLoop
        .P(1.0)
        .I(0.0)
        .D(0.0)
        .OutputRange(-1.0, 1.0);

      elevator_master_config.closedLoop.maxMotion
        .MaxVelocity(6000.0)
        .MaxAcceleration(6000.0)
        .AllowedClosedLoopError(5.0);
        
      m_MasterElevatorMotor.Configure(elevator_master_config, SparkMax::ResetMode::kResetSafeParameters, SparkMax::PersistMode::kPersistParameters);

      
      MotorInitVictor(m_CoralMotor);


      // We need to run our vision program in a separate thread.
      // If not run separately (in parallel), our robot program will never
      // get to execute.
      std::thread visionThread( VisionThread );

      visionThread.detach();
    }

    void AutonomousInit() override {

      headingController.EnableContinuousInput(-M_PI, M_PI);

      //auto code goes here
      if (traj_frfr.has_value()) {
        // Get the initial pose of the trajectory
        if (auto initialPose = traj_frfr.GetInitialPose(IsRedAlliance())) {
          // Reset odometry to the start of the trajectory
          std::cout << "Resetting Gyro" << std::endl;
          m_swerve.ResetPose(initialPose.value());
        }
      }

      // Reset and start the timer when the autonomous period begins
      timer.Restart();
      //timer.Stop //timer affects 
    }

    void AutonomousPeriodic() override {
      
      if (traj_frfr.has_value()) {
        // Sample the trajectory at the current time into the autonomous period
        if (auto sample = traj_frfr.value().SampleAt(timer.Get(), IsRedAlliance())) {
            FollowTrajectory(sample.value());
        } else {
          std::cout << "No more samples!" << std::endl;
        }
        
        int splits = sizeof(traj_frfr.value().splits)/sizeof(int);
        // int is 4 bytes

        frc::Pose2d rpose = m_swerve.m_poseEstimator.GetEstimatedPosition();
        

        static int last_timer = 0;
        if (last_timer != timer.Get().value()) {
          std::cout << "x" << rpose.X().value() << " y" << rpose.Y().value() << " r" << rpose.Rotation().Degrees().value() << std::endl;
        } 
        last_timer = timer.Get().value();

      }
      m_swerve.UpdateOdometry();
    }

    void TestInit() override {

    }
    void TestPeriodic() override {
      
    }

    void TeleopInit() override {
      //initialize teleop-specific motors/sensors/etc. here
    }

    void TeleopPeriodic() override {
      // m_swerve.m_frontLeft.GetPosition();
      // double leftDistance = leftEncoder.GetDistance();
      // double rightDistance = rightEncoder.GetDistance();
      // std::cout << "Left Encoder Distance: " << leftDistance << std::endl;
      // std::cout << "Right Encoder Distance: " << rightDistance << std::endl;

      frc::Pose2d rpose = m_swerve.m_poseEstimator.GetEstimatedPosition();
        

      std::cout << "x" << rpose.X().value() << " y" << rpose.Y().value() << " r" << rpose.Rotation().Degrees().value() << std::endl;



      DriverControls(true);
      OperatorControls();
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

    void DriverControls(bool fieldRelative) {
      // SLOOOOOOWMODE
      // low speed by 0.3 if holding r-trigger //0.2
      double lowGear = m_driverController.GetRightTriggerAxis() > 0.1 ? 0.3 : 1.0;

      // Get the x speed. We are inverting this because Xbox controllers return
      // negative values when we push forward.
      auto xSpeed = -m_xspeedLimiter.Calculate(
                              frc::ApplyDeadband(m_driverController.GetLeftY(), 0.2)) *
                          Drivetrain::kMaxSpeed * lowGear;
      //auto xSpeed = (units::velocity::meters_per_second_t) 0;

      // Get the y speed or sideways/strafe speed. We are inverting this because
      // we want a positive value when we pull to the left. Xbox controllers
      // return positive values when you pull to the right by default.
      auto ySpeed = -m_yspeedLimiter.Calculate(
                              frc::ApplyDeadband(m_driverController.GetLeftX(), 0.2)) *
                          Drivetrain::kMaxSpeed * lowGear;
      // auto ySpeed = (units::velocity::meters_per_second_t) 0;

      // Get the rate of angular rotation. We are inverting this because we want a
      // positive value when we pull to the left (remember, CCW is positi std::cout << "Wheel angles FL,FR/BL,BR:"
              
      // mathematics). Xbox controllers return positive values when you pull to
      // the right by default.
      auto rot = -m_rotLimiter.Calculate(
                          frc::ApplyDeadband(m_driverController.GetRightX(), 0.2)) *
                      Drivetrain::kMaxAngularSpeed * lowGear;

      auto atData = GetATagVariables();
      


      std::cout << "BL" << m_swerve.m_backLeft.GetPosition().angle.Degrees().value()
                << " BR" << m_swerve.m_backRight.GetPosition().angle.Degrees().value()
                << " FL" << m_swerve.m_frontLeft.GetPosition().angle.Degrees().value()
                << " FR" << m_swerve.m_frontRight.GetPosition().angle.Degrees().value()
                << std::endl;






    // Relative Switch (A)
    fieldRelative = !m_driverController.GetAButton();

    // Reset Gyro and Position (Y)
    if (m_driverController.GetYButton()) {
      m_swerve.Reset();
    }

    if (m_driverController.GetXButton()) {
      xSpeed = Drivetrain::kMaxSpeed / 4;
    }



    // cycle through tags
    if (m_driverController.GetStartButtonPressed()) {
      selectedTag++;
      if (selectedTag > 11) selectedTag = 1;
    }

    // Look To April Tag (Left Bumper)
    if (m_driverController.GetLeftBumper()) {
      rot = -atData.radsToTurn*10;
      fieldRelative = true;
      // std::cout << (double) faceAprilTag
      //           << std::endl;
    }


    // Pressed once
    if (m_driverController.GetRightBumperButtonPressed()) {
      atPreviousError = 0;
      atIntegral = 0;
    }

    // Held bumper. Hunt and pounce (predator alignment) April Tag (Right Bumper)
    if (m_driverController.GetRightBumper()) {


      if (std::abs(atData.headOnOffsetDeg) > 0.05) {
        PIDReturn VisionPIDReturn = VisionPIDController(atData.headOnOffsetDeg, atPreviousError, atIntegral);
        ySpeed = (units::velocity::meters_per_second_t) -VisionPIDReturn.PIDReturnValue;

        std::cout << "PID Return: " << VisionPIDReturn.PIDReturnValue << std::endl;

        atPreviousError = VisionPIDReturn.previousError;
        atIntegral = VisionPIDReturn.integral;
      } else {
        // When we get aligned, start moving towards tag
        std::cout << "Lined up" << std::endl;
        xSpeed = (units::velocity::meters_per_second_t) atData.needToMoveDist * 2.0;
      }
      
      

      
      rot = -atData.radsToTurn*10; // turn robot to face tag

      fieldRelative = true;
    }

    
    // Brake (B) and drive
    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, m_driverController.GetBButton());

    } //End DriveControls

    void OperatorControls() {
      // Adjust sideshift + elevator
      
      // Elevator
      //std::cout << m_operatorController.GetPOV() << std::endl;
      
      switch (m_operatorController.GetPOV()) {
        case 0: { // up
          std::cout << "Elevator up" << std::endl;
          m_ElevatorController.SetReference(150.0, SparkBase::ControlType::kPosition, rev::spark::kSlot0); //180 "max"
          //m_MasterElevatorMotors
          break;
        }
        case 45: {
          //std::cout << m_MasterElevatorMotor.GetAbsoluteEncoder().GetPosition() << std::endl;
          break;
        }
        case 90: { // right
          std::cout << "Elelator 3" << std::endl;
          m_ElevatorController.SetReference(100.0, SparkBase::ControlType::kPosition, rev::spark::kSlot0);
          break;
        }
        case 135: {
          break;
        }
        case 180: { // down
          std::cout << "Elevator down" << std::endl;
          m_ElevatorController.SetReference(0.0, SparkBase::ControlType::kPosition, rev::spark::kSlot0);
          break;
        }
        case 225: {
          break;
        }
        case 270: { // left
          std::cout << "elevator 2" << std::endl;
          m_ElevatorController.SetReference(50.0, SparkBase::ControlType::kPosition, rev::spark::kSlot0);
          break;
        }
        case 315: {
          break;
        }


        default: {
          break;
        }
      }


      // Coral Motor
      if (m_operatorController.GetBButton()) {
        //m_CoralMotor.SetVoltage((units::volt_t) 12.0);
        m_CoralMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1.0);
      } else {
        m_CoralMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
      }

      // if (m_operatorController.GetAButton()) {
      //   m_MasterElevatorMotor.SetVoltage(units::volt_t{6.0});
      // } else {
      //   m_MasterElevatorMotor.SetVoltage(units::volt_t{0.0});
      // }

      if (m_operatorController.GetYButton()) {
        // m_ElevatorController.SetIAccum()
      }

    }


    // OTHER FUNCTIONS



    struct PIDReturn {
      double PIDReturnValue;
      double previousError;
      double integral;
    };

    /**
     * processVariable = Yaw/Y axis rotation
     * previousError = 0 initial, last error next times. how far away we were on the last loop
     * integral = 0 initial, feed it from returns
     */
    PIDReturn VisionPIDController(double processVariable, double previousError, double integral) {
      double setpoint = 0;
      double Kp = 1; //1 is cool, first to chage
      double Ki = 0; //0.1 too high?
      double Kd = 0.1; //0.1 is cool, 2nd to change

      double error = setpoint - processVariable;
      integral += error;
      double derivative = error - previousError;
      previousError = error;
      double returnVal = Kp * error + Ki * integral + Kd * derivative;


      

      PIDReturn returnValues = {
        returnVal,
        previousError,
        integral,
      };
      return returnValues;
    }





    struct ATagVars {
      units::angular_velocity::radians_per_second_t radsToTurn;
      double needToMoveDist;
      double headOnOffsetDeg;
    };
    ATagVars GetATagVariables() {
      ATagVars atData;

      gyroYawHeading = m_swerve.GetYaw();
      gyroYawRate = m_swerve.GetRate();

      double dEventualYaw = (double) gyroYawHeading + (0.5 / 600.0) * (double) gyroYawRate * std::abs((double) gyroYawRate); // accounts for overshooting
      //find the shortest degrees to face tag
      int degreesToTurn = (int) ((double) dEventualYaw - desiredYaw) % 360;
      if (degreesToTurn > 180) degreesToTurn -= 360;
      if (degreesToTurn < -180) degreesToTurn += 360;
      // Converts degrees from vision thread to radians per second.
      units::angular_velocity::radians_per_second_t radsToTurn =
       (units::angular_velocity::radians_per_second_t) degreesToTurn * M_PI / 180;
      
      
      atData.radsToTurn = radsToTurn;
      atData.headOnOffsetDeg = headOnOffsetDeg;
      atData.needToMoveDist = needToMoveDist;
      
      return atData;
    }


  
    bool IsRedAlliance() {
      auto alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::kBlue);
      return alliance == frc::DriverStation::kRed;
    }



  // AUTONOMOUS



    void FollowTrajectory(const choreo::SwerveSample& sample) {
        // Get the current pose of the robot
        frc::Pose2d pose = m_swerve.m_poseEstimator.GetEstimatedPosition();


        // Calculate feedback velocities
        units::meters_per_second_t xFeedback{xController.Calculate(pose.X().value(), sample.x.value())};
        std::cout << "X position is " << pose.X().value() << " and sample position is " << sample.x.value() << " so feedforward is " << xFeedback.value();
        units::meters_per_second_t yFeedback{yController.Calculate(pose.Y().value(), sample.y.value())};
        units::radians_per_second_t headingFeedback{
            headingController.Calculate(pose.Rotation().Radians().value(), sample.heading.value())
        };
        std::cout << " Sample vx is " << sample.vx.value();
        // Generate the next speeds for the robot
        frc::ChassisSpeeds speeds{
            sample.vx + xFeedback,
            sample.vy + yFeedback,
            sample.omega + headingFeedback
        };
        
        if (abs( speeds.vx.value() )  < 0.001) {
            speeds.vx = (units::meters_per_second_t) 0.0;
        }
        if (abs( speeds.vy.value() )  < 0.001) {
            speeds.vy = (units::meters_per_second_t)0.0;
        }
        if (abs( speeds.omega.value() )  < 0.001) {
            speeds.omega = (units::radians_per_second_t)0.0;
        }
        // Apply the generated speeds
        std::cout << " New speed is " << speeds.vx.value() << std::endl;
        m_swerve.Drive(speeds.vx, speeds.vy, speeds.omega, true);
    };




};





int main() {
  return frc::StartRobot<Robot>();
}

