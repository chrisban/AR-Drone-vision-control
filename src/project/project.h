#ifndef __HEADER_PROJECT_H__
#define __HEADER_PROJECT_H__

#define KEY_DOWN(key) (GetAsyncKeyState(key) & 0x8000)
#define KEY_PUSH(key) (GetAsyncKeyState(key) & 0x0001)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "..\ardrone\ardrone.h"

const int		StateInitState = 0x00,
						StateInitHover = 0x01,
						StateLanding = 0x03,
						StateSetAltitude = 0x04,

						StateFCSearching = 0x11,
						StateFCMoving = 0x12,
						StateFCLostTarget = 0x13,
						StateFCAiming = 0x14,
						StateFCFollowing = 0x16, 
						StateFCBreaking = 0x17,

						StateBTSearching = 0x21,
						StateBTMoving = 0x22,
						StateBTForwardHover = 0x25;

struct Point;
struct Location;
struct Movement;
class Tracking;

void cvSetSize(int pos);
void cvSetFactor(int pos);
void cvErodeImage(IplImage *source, IplImage *destination);
void cvDilateImage(IplImage *source, IplImage *destination);
void cvSetupTrackbar();

Location cvSetTargetCenter(IplImage *source, IplImage *destination);

void fprint(const char *format, ...);
void consolePrint(int leftPad, const char *label, const char *type, ...);
void home();
void keypress(Tracking &tracking);

class PointAverager {
public:
	CvPoint3D64f XYZ(int x, int y, float z);
	void print();
	void clear();

	CvPoint3D64f lastAverage;
	
private:
	int index;
	int size;
	int x[50];
	int y[50];
	double z[50];
};

struct Point {
public:
	int x;
	int y;
};

struct Location {
	// CvPoint2D64f.x is the image vertical value or the height of the target for the forward camera 
	//										 or the forward/backward for the down camera
	// CvPoint2D64f.y is the image horizontal value or the left/right of center the target is in the image
	Point location;
	
	// 0 = not found; 1 = target is far away; 256 = robot is touching target
	float distance;
	
	// The angle of rotation of the target in relation to the ARDrone's down cam. IE: If the target 
	// is facing NW and the ARDrone is facing N then the direction would be 135°. This means that to
	// get the angle for change of direction the ARDrone will need to subtract 90° for the real angle
	// of change which is 45°. A negative value means no direction.
	// Additionally: (duplicate definition)
	// direction is the angle the target is looking
	// ie: If the target is looking NW (the front of the hat is aimed towards the top left
	// of the captured image frame) then direction would return 135.0°. If the target was facing
	// E (or to the right edge of the captured image frame) it would be 0.0°. This value is a
	// double to allow for more accurate angles. 
	//             90.0°
	//              ^
	//              |
	//              |
	// 180.0° <-----+-----> 0.0°
	//              |
	//              |
	//             \/
	//      270.0° or -90.0°
	double direction;

	bool isFound;
	
	void print();
};

struct Movement {
	struct Orientation{
		// Orientation
		double roll, pitch, yaw;
	};

	struct Velocity{				
		// Velocity
		double vx, vy, vz;
		double velocity;
	};

	Orientation orientation;

  // Altitude
  double altitude;
				
	Velocity speed;

  // Battery
  int battery;

	void print();
};

// Tracking Class
class Tracking {
public:
	PointAverager averager;
	ARDrone	ardrone;
	bool	isVersion20, isSearching, changeState;
	Location	target;
	Movement	movement;

	int width;
	int height;

  Tracking();                             // Constructor
  ~Tracking();                            // Destructor

	void findTargetUsingForwardCamera(IplImage *image);	// Find the target in the given image
	void findTargetUsingDownCamera(IplImage *image);	// Find the target in the given image
	
	void	stateMachine();
	int		getCurrentState();
	void	setState(int newState);

	void	doMovement();
	void	followTarget(int j);

	void	rotate();
	void	rotate(float angle);

	void	swoop();

	void	print();
	CvPoint3D64f	setTarget(int x, int y, float distance, double direction);
private:
	int _currentState;
	int _lastState;
	
	float getVerticalSpeed();

	double  getRotationAngle();
	double  getAltitudeAngle();
	double	getForwardAngle();
	double	getSideAngle();
	
	void cvSetTargetCenter(IplImage *source, IplImage *destination);
	void privateTargetFindingFunction(IplImage *image);
	void	updateMovement();
};

#endif