// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Drivetrain.h"

// #include "AprilTagDetection.h"

#include <cameraserver/CameraServer.h>
#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
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
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>



// #include <frc/Encoder.h>
// #include <frc/smartdashboard/SmartDashboard.h>

//Robot Pos Variables
static units::angle::degree_t gyroYawHeading; //robot yaw (degrees)
static units::angular_velocity::degrees_per_second_t gyroYawRate; //robot rotate rate (degrees/second)
static frc::Pose2d pose;

//Camera Variables
cs::UsbCamera camera1;
static cs::CvSink cvSink;

//April Tag Variables
static double desiredYaw;
static double needToMoveDist;
static double headOnOffsetDeg; // how off are we from being headon with the tag?

//PID idk
static double atPreviousError;
static double atIntegral;


class Robot : public frc::TimedRobot {


static void VisionThread() {
  frc::AprilTagDetector detector;
  

  // look for tag36h11, correct 3 error bits
    detector.AddFamily("tag36h11", 0);

    // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
    // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
    frc::AprilTagPoseEstimator::Config poseEstConfig = {
        .tagSize = units::length::inch_t(6.5),
        .fx = 699.3778103158814,
        .fy = 677.7161226393544,
        .cx = 345.6059345433618,
        .cy = 207.12741326228522};
    frc::AprilTagPoseEstimator estimator(poseEstConfig);

    // Get the USB camera from CameraServer
    camera1 = frc::CameraServer::StartAutomaticCapture(0); //Note Camera
    //camera2 = frc::CameraServer::StartAutomaticCapture(1); //April Tag Camera

    // Set the resolution
    camera1.SetResolution(640, 480); //160, 120 //320, 240
    //camera2.SetResolution(640, 480);

    // Get a CvSink. This will capture Mats from the Camera
    auto cvSink = frc::CameraServer::GetVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    cs::CvSource outputStream =
        frc::CameraServer::PutVideo("Detected", 640, 480);
    
    cvSink.SetSource(camera1); //camera2

    // Mats are very memory expensive. Lets reuse this Mat.
    cv::Mat mat;
    cv::Mat grayMat;
    cv::Mat hsvMat;

    // Instantiate once
    std::vector<int64_t> tags;
    // Outline colors in bgr, not rgb
    cv::Scalar outlineColor{0, 255, 0}; // green
    cv::Scalar outlineColorTop{0, 0, 255}; // red
    cv::Scalar outlineColorBottom{255, 0, 0}; // blue
    cv::Scalar crossColor{0, 0, 255};

    // We'll output to NT
    auto tagsTable =
        nt::NetworkTableInstance::GetDefault().GetTable("apriltags");
    auto pubTags = tagsTable->GetIntegerArrayTopic("tags").Publish();

    while (true) {
      // Tell the CvSink to grab a frame from the camera and
      // put it in the source mat.  If there is an error notify the
      // output.
      // grab robot yaw at frame grab

      units::angle::degree_t gyroYawHeadingLocal = gyroYawHeading;
      if (cvSink.GrabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.NotifyError(cvSink.GetError());
        // skip the rest of the current iteration
        continue;
      }

      cv::cvtColor(mat, grayMat, cv::COLOR_BGR2GRAY);

      cv::Size g_size = grayMat.size();
      frc::AprilTagDetector::Results detections =
          detector.Detect(g_size.width, g_size.height, grayMat.data);

      // have not seen any tags yet
      tags.clear();


      //desiredYaw = 0; // if this is enabled, robot will not remember tag location if it leaves the field of view
      needToMoveDist = 0.0;
      //std::cout << "i see a tag " << detections.size() << std::endl;

      for (const frc::AprilTagDetection* detection : detections) {
        // remember we saw this tag
        tags.push_back(detection->GetId());

        // draw lines around the tag
        for (int i = 0; i <= 3; i++) {
          int j = (i + 1) % 4;
          const frc::AprilTagDetection::Point pti = detection->GetCorner(i);
          const frc::AprilTagDetection::Point ptj = detection->GetCorner(j);
          // draws sides different colors
          switch (i) {
            case 0: {
              //bottom
              line(mat, cv::Point(pti.x, pti.y), cv::Point(ptj.x, ptj.y),
              outlineColorBottom, 2);
              break;
            }
            case 2: {
              //top
              line(mat, cv::Point(pti.x, pti.y), cv::Point(ptj.x, ptj.y),
              outlineColorTop, 2);
              break;
            }
            default: {
              line(mat, cv::Point(pti.x, pti.y), cv::Point(ptj.x, ptj.y),
              outlineColor, 2);
            }
          }
        }

        // mark the center of the tag
        const frc::AprilTagDetection::Point c = detection->GetCenter();
        int ll = 10;
        line(mat, cv::Point(c.x - ll, c.y), cv::Point(c.x + ll, c.y),
             crossColor, 2);
        line(mat, cv::Point(c.x, c.y - ll), cv::Point(c.x, c.y + ll),
             crossColor, 2);

        // identify the tag
        int tagId = detection->GetId();
        putText(mat, std::to_string(tagId),
                cv::Point(c.x + ll, c.y), cv::FONT_HERSHEY_SIMPLEX, 1,
                crossColor, 3);

        // determine pose
        frc::Transform3d pose = estimator.Estimate(*detection);

        // put pose into NT
        frc::Rotation3d rotation = pose.Rotation();
        tagsTable->GetEntry(fmt::format("pose_{}", detection->GetId()))
            .SetDoubleArray(
                {{ pose.X().value(),
                   pose.Y().value(),
                   pose.Z().value(),
                   rotation.X().value(),
                   rotation.Y().value(),
                   rotation.Z().value() }});



        // Get angle we're off from being head-on with the tag (0 degrees)
        double tagHeadOnDegs[3] = { rotation.X().value(), rotation.Y().value(), rotation.Z().value() };
        //auto tagHeadOnDeg = tagsTable->GetEntry("apriltags/pose_X")

        std::cout << "Tag ID: " << detection->GetId() << " | X: " << tagHeadOnDegs[0] << " | Y: " << tagHeadOnDegs[1] << " | Z: " << tagHeadOnDegs[2]
          << std::endl;
        double tagRotDist = (g_size.width/2)-c.x; //tag distance from center
        double tagRotDistDeg = tagRotDist / 10; // tag distance in degrees, roughly
        
        units::length::meter_t tagDist = pose.Z(); //robot distance from tag

        // How far we want to be from the tag
        double targetDist;

        //tag rotation
        //units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // calculates fixed tag location relative to initial gyro rotation
        //desiredYaw = (double) tagBearing; // writes desired yaw to be tag location
        
        if (frc::DriverStation::kRed == frc::DriverStation::GetAlliance()) {
          switch (tagId) {
            case 6: {
              
              // Curious Alignment
              // Tag distance (Z-Axis)
              targetDist = 2.642; // Distance we want to be from april tag
              needToMoveDist = (double) tagDist - targetDist;
              // Tag rotation
              units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // Calculates fixed tag location relative to initial gyro rotation
              desiredYaw = (double) tagBearing; // Writes desired yaw to be tag location
              
              // Predator Alignment
              // Tag headon
              headOnOffsetDeg = tagHeadOnDegs[1];
              
              break;
            }
          }


        } else if (frc::DriverStation::kBlue == frc::DriverStation::GetAlliance()) {
          switch (tagId) {
            case 7: {
              //printf("saw tag 7 (Blue team)/n");
              // Tag distance (z axis)
              targetDist = 2.642; // Distance we want to be from april tag
              needToMoveDist = (double) tagDist - targetDist;
              // Tag rotation
              units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // Calculates fixed tag location relative to initial gyro rotation
              desiredYaw = (double) tagBearing; // Writes desired yaw to be tag location
              break;
            }
          }
        }

      }




      //put list of tags onto NT
      pubTags.Set(tags);

      // Give the output stream a new image to display
      outputStream.PutFrame(mat);
    }
}


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
      
      // We need to run our vision program in a separate thread.
      // If not run separately (in parallel), our robot program will never
      // get to execute.
      std::thread visionThread( VisionThread );
      visionThread.detach();
    }

    void AutonomousInit() override {
        //auto code goes here
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
      //double lowGear = m_driverController.GetRightTriggerAxis() > 0.1 ? 0.3 : 1.0;

      // Get the x speed. We are inverting this because Xbox controllers return
      // negative values when we push forward.
      auto xSpeed = -m_xspeedLimiter.Calculate(
                              frc::ApplyDeadband(m_driverController.GetLeftY(), 0.2)) *
                          Drivetrain::kMaxSpeed; //* lowGear;
      //auto xSpeed = (units::velocity::meters_per_second_t) 0;

      // Get the y speed or sideways/strafe speed. We are inverting this because
      // we want a positive value when we pull to the left. Xbox controllers
      // return positive values when you pull to the right by default.
      auto ySpeed = -m_yspeedLimiter.Calculate(
                              frc::ApplyDeadband(m_driverController.GetLeftX(), 0.2)) *
                          Drivetrain::kMaxSpeed; //* lowGear;
      // auto ySpeed = (units::velocity::meters_per_second_t) 0;

      // Get the rate of angular rotation. We are inverting this because we want a
      // positive value when we pull to the left (remember, CCW is positi std::cout << "Wheel angles FL,FR/BL,BR:"
              
      // mathematics). Xbox controllers return positive values when you pull to
      // the right by default.
      auto rot = -m_rotLimiter.Calculate(
                          frc::ApplyDeadband(m_driverController.GetRightX(), 0.2)) *
                      Drivetrain::kMaxAngularSpeed; //* lowGear;

      auto atData = GetATagVariables();
      


    // Relative Switch (A)
    fieldRelative = !m_driverController.GetAButton();

    // Reset Gyro and Position (Y)
    if (m_driverController.GetYButton()) {
      m_swerve.Reset();
    }

    if (m_driverController.GetXButton()) {
      xSpeed = Drivetrain::kMaxSpeed / 4;
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

    // Held. Hunt and pounce (predator alignment) April Tag (Right Bumper)
    if (m_driverController.GetRightBumper()) {
      // if (atData.headOnOffsetDeg > 0.05 ) { // 0.005
      //   ySpeed = (units::velocity::meters_per_second_t) 0.1;
      // } else if (atData.headOnOffsetDeg < -0.05) { // 0.005
      //   ySpeed = (units::velocity::meters_per_second_t) -0.1;
      // } else { // otherwise, if we are aligned with tag, then start moving towards it
      //   std::cout << "Lined up" << std::endl;
      // } 


      if (std::abs(atData.headOnOffsetDeg) > 0.05) {
        PIDReturn VisionPIDReturn = VisionPIDController(atData.headOnOffsetDeg, atPreviousError, atIntegral);
        ySpeed = (units::velocity::meters_per_second_t) -VisionPIDReturn.PIDReturnValue;
        std::cout << "PID Return: " << VisionPIDReturn.PIDReturnValue << std::endl;
        atPreviousError = VisionPIDReturn.previousError;
        atIntegral = VisionPIDReturn.integral;
      }
      
      

      
      rot = -atData.radsToTurn*10; // turn robot to face tag

      fieldRelative = true;
    }

    
    // Brake (B) and drive
    m_swerve.Drive(xSpeed, ySpeed, rot, fieldRelative, m_driverController.GetBButton());

    } //End DriveWithJoystick


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


  





};





int main() {
  return frc::StartRobot<Robot>();
}

