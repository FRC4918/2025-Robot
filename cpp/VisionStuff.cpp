#include "VisionStuff.h"

#include <cameraserver/CameraServer.h>
#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
#include <iostream>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>



cs::UsbCamera camera1;
units::angle::degree_t gyroYawHeading;
int selectedTag;

void VisionThread() {
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
        .cy = 207.12741326228522
    };
    frc::AprilTagPoseEstimator estimator(poseEstConfig);

    // Get the USB camera from CameraServer
    camera1 = frc::CameraServer::StartAutomaticCapture(0); //Note Camera
    //camera2 = frc::CameraServer::StartAutomaticCapture(1); //April Tag Camera

    // Set the resolution
    camera1.SetResolution(480, 360); //160, 120 //320, 240 //640, 480
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
    cv::Scalar selectedColor{0, 255, 255};

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

        // identify the tags::UsbCamera camera, units::angle::degree_t gyroYawHeading, int currentSelectedTag
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


        cv::putText(mat, std::to_string(selectedTag), 
          cv::Point(30, 30), cv::FONT_ITALIC, 0.5, selectedColor, 2);
        

        // Only update tag vars if we select the detected tag
        if (detection->GetId() == selectedTag) {
        //draw selected thing
        cv::circle(mat, cv::Point(c.x, c.y), 10, selectedColor, 3);


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


        // Set global variables to use april tag data

        // Curious Alignment
        // Tag distance (Z-Axis)
        targetDist = 0.5; // Distance we want to be from april tag
        needToMoveDist = (double) tagDist - targetDist;
        // Tag rotation
        units::angle::degree_t tagBearing = gyroYawHeadingLocal + (units::angle::degree_t) tagRotDistDeg; // Calculates fixed tag location relative to initial gyro rotation
        desiredYaw = (double) tagBearing; // Writes desired yaw to be tag location
        
        // Predator Alignment
        // Tag headon
        headOnOffsetDeg = tagHeadOnDegs[1];



        

        //specific tag adjustments (tag switch case)
        switch (detection->GetId()) {
          // player/coral stations
          case 1: // red
          case 2:
          case 12: // blue
          case 13: {
            // align to collect
            break;
          }

          // reefs
          case 6: // red
          case 7:
          case 8:
          case 9:
          case 10:
          case 11:
          case 17: // blue
          case 18:
          case 19:
          case 20:
          case 21:
          case 22: {
            // align elevator with coral acceptor things
            break;
          }

          default: {

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