#include <cameraserver/CameraServer.h>
#include <units/angle.h>

extern cs::UsbCamera camera1;
extern units::angle::degree_t gyroYawHeading;
extern int selectedTag;

//April Tag Variables
extern double desiredYaw;
extern double needToMoveDist;
extern double headOnOffsetDeg; // how off are we from being headon with the tag?


void VisionThread();