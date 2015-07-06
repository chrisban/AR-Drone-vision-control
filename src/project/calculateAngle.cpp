#include "project.h"

// Coodinate system
//   Front of the AR.Drone is X-axis, left is Y-axis, upper is Z-axis.
//   Also front is 0.0 [rad], each axis CCW is positive.
//            X
//           +^-
//            |
//            |
//    Y <-----+ (0,0)
//            Z
//
// Since the coordinates from the camera are with 0,0 in the bottom right of the image and the returned
// values are with the 0,0 in the center of the image, the final calculation will need to be offset by
// half the window frame so that the results can be Z+ for up, Z- for down, Y+ for left, Y- for right, 
// X+ for forward, X- for backward.
//             X+
//             ^
//             |
//             |
//    Y+ <-----+-----> Y-
//             |(0,0)
//             |
//            \/X-

double frontRotationAngle = 1280.0 / 90.0;

// ------------------------------------------------------------------------------------
// Tracking::getRotationAngle()
// Based on the horizontal position (X) from the center of the camera, the angle of the
// lens (90°) and size of the image (720p), the angle the target is off center is the 
// angle of rotation to center the ARDrone to the object.
// ------------------------------------------------------------------------------------
double Tracking::getRotationAngle()
{
	return target.location.x / frontRotationAngle; // 1280 pixels / 90 degrees
}

// ------------------------------------------------------------------------------------
// Tracking::getAltitudeAngle()
// Based on the vertical position (Y) from the center of the camera, the angle of the
// lens (90°) and size of the image (720p), the angle the target is off center is the 
// angle of altitude change to center the ARDrone to the object.
//
// NOTE: This angle implys that the ARDrone will be level with the target and will need
// to have some hight adjustment so that it will be above the target so it does not hit
// or touch the target.
// ------------------------------------------------------------------------------------
double Tracking::getAltitudeAngle()
{
	return target.location.y / 8.0; // 720 pixels / 90 degrees
}

// ------------------------------------------------------------------------------------
// Tracking::getForwardAngle()
// Based on the vertical (forward/backward) position (Y) from the center of the camera, 
// the angle of the lens (90°) and size of the image (720p), the angle the target is 
// off center is the angle of altitude change to center the ARDrone to the object.
// ------------------------------------------------------------------------------------
double Tracking::getForwardAngle()
{
	return target.location.y / 3.75; // 240 pixels / 64 degrees
}

// ------------------------------------------------------------------------------------
// Tracking::getSideAngle()
// Based on the horizontal (left/right) position (X) from the center of the camera, 
// the angle of the lens (90°) and size of the image (720p), the angle the target is 
// off center is the angle of rotation to center the ARDrone to the object.
// ------------------------------------------------------------------------------------
double Tracking::getSideAngle()
{
	return target.location.x / 5.0; // 320 pixels / 64 degrees
}
