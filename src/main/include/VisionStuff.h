#pragma once

#include <cameraserver/CameraServer.h>
#include <units/angle.h>
#include <units/length.h>

extern cs::UsbCamera camera1;
extern units::angle::degree_t gyroYawHeading;
//extern int selectedTag;

//April Tag Variables
extern double desiredYaw;
extern units::length::meter_t needToMoveDist;
extern double headOnOffsetDeg; // how off are we from being headon with the tag?



void VisionThread();