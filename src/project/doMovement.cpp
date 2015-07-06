#include "project.h"
//#include "ardrone\ardrone.h"

extern FILE *pFile;

// These values are arbitrary and will need to be modified with testing. 
#define SmallXError 10
#define MediumXError 25
#define LargeXError 40

#define SmallYError 10
#define MediumYError 25
#define LargeYError 40


float AND(float a, float b);
float OR(float a, float b);

float AND(float a, float b)	// return smallest
{
	if (a < b) return a;
	else return b;
}

float OR(float a, float b)	// return largest
{
	if (a > b) return a;
	else return b;
}

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


// Since the coordinates from the camera are with 0,0 in the bottom right of the image and the returned
// values are with the 0,0 in the center of the image, the final calculation will need to be offset by
// half the window frame so that the results can be Z+ for up, Z- for down, Y+ for left, Y- for right, 
// X+ for forward, X- for backward.
//             X+
//             ^
//             |
//             |
//    Y- <-----+-----> Y+
//             |(0,0)
//             |
//            \/X-

float Fuzzify(float Error, float ShortError, float MediumError, float LongError)
{

	float Slight = 0.0f; // Target is close by
	float Medium = 0.0f; // Target is not near
	float Large = 0.0f; // Target is far away
	float NothingY = 0.0f; // Target is being followed or can not be found
	
	float MAX = 255.0f;

	if (Error > LongError)
	{
		Large = MAX;
	}
	else if(Error > MediumError)
	{
		Large = (MAX*(Error+MediumError))/(LongError-MediumError);
		Medium = MAX - Large;
	}
	else if( Error > ShortError)
	{
		Medium = (MAX*(Error+ShortError))/(MediumError-ShortError);
		Slight = MAX - Medium;
	}
	else if(Error > 0)
	{
		Slight = (MAX*(Error+0.0f))/(ShortError-0.0f);
		NothingY = MAX - Slight;
	}
	else
	{
		NothingY = MAX;
	}

	// Changing the multipliers will change the speed.
	// The varriable / 255 gives a fraction of 1
	// The multiplier is then the max speed possible;
	// as SlightForward/255 decreases the speed also decreases
	Slight = (Slight / MAX) * 0.1f;
	Medium = (Medium / MAX) * 0.15f;
	Large = (Large / MAX) * 0.2f;

	//printf("Er = %3.2f, Low = %3.2f, Med = %3.2f, Lrg = %3.2f\n", Error, Slight, Medium, Large);

	float Forward = OR(Slight, Medium);
	Forward = OR(Forward, Large);

	return Forward;
}

// 0 ft - 92
// 5 ft - 80
// 20 ft - 19

float ShortD = 8.5f;//75.0f;
float MediumD = 10.5f;//45.0f;
float LongD = 15.0f;

float SmallDV = 3.0f;
float MediumDV = 15.0f;
float LargeDV = 25.0f;

float FuzzifyDistanceApproach(float Error, float ShortError, float MediumError, float LongError)
{
	// Theortically finished
	// control over 0-7 make 5 center;

	// 0 ft - 92
	// 5 ft - 80
	// 20 ft - 19

	/*float LongD = 19.0f;
	float MediumD = 45.0f;
	float ShortD = 75.0f;
	*/

	float Slight = 0.0f; // Target is close by
	float Medium = 0.0f; // Target is not near
	float Large = 0.0f; // Target is far away
	float NothingY = 0.0f; // Target is being followed or can not be found
	
	float MAX = 255.0f;

	if (Error > LongError)
	{
		Large = MAX;
	}
	else if(Error > MediumError)
	{
		Large = (MAX*(Error+MediumError))/(LongError-MediumError);
		Medium = MAX - Large;
	}
	else if( Error > ShortError)
	{
		Medium = (MAX*(Error+ShortError))/(MediumError-ShortError);
		Slight = MAX - Medium;
	}
	else
	{
		NothingY = MAX;
	}

	// Changing the multipliers will change the speed.
	// The varriable / 255 gives a fraction of 1
	// The multiplier is then the max speed possible;
	// as SlightForward/255 decreases the speed also decreases
	Slight = (Slight / MAX) * 0.08f;
	Medium = (Medium / MAX) * 0.12f;
	Large = (Large / MAX) * 0.2f;

	//printf("Er = %3.2f, Low = %3.2f, Med = %3.2f, Lrg = %3.2f\n", Error, Slight, Medium, Large);

	float Forward = OR(Slight, Medium);
	Forward = OR(Forward, Large);

	//printf("Distance estimate");
	// DRM TEST
	

	return Forward;

}

float getRotation(float horizontal_offset)
{
	float range = 1.0f;
	// Rotation speed in Rot/Sec in the positive direction.
	return (horizontal_offset > 0.0 ? range : -range);
}

float Tracking::getVerticalSpeed()
{
	float altitude = ardrone.getAltitude();// >= 1.6f);
	float speed;

	if(altitude > 1.72f)
	{
		speed = -0.5f;
	}
	else if(altitude < 1.65f)
	{
		speed = 0.5f;
	}
	else
	{
		speed = 0.0f;
	}

	return speed;
}

// ------------------------------------------------------------------------------------
// Tracking::findTargetUsingForwardCamera()
// Find the targets location on a video feed using the forward camera image
// Return value LOCATION with X, Y, and Distance
// Where X is the vertical offset of the target
//		 Y is the horizontal offset of the target
//		 Distance represents an estimation from 1 to 256 for how far away the object is 
//			or 0 when not found
// ------------------------------------------------------------------------------------
void Tracking::doMovement() // move point a to point b
{
	float forward = 0.0f;
	float rotation = 0.0f;
	float VerticalDelta = 0.0f;

	// Modification Neccisary Distance changed.
	//if(target.distance >= 3)
	//	forward = Fuzzify((float)target.distance, ShortD, MediumD, LongD);// 0 no object range between 0 and 20 (maximum) - feet

	forward = FuzzifyDistanceApproach(target.distance, ShortD, MediumD, LongD);
	
	int absoluteX = abs(target.location.x);
	int window = width / 7.5;				//width / 6.5;

	fprint("- Rotate --> Dis = %3.2f\n             Abs = %d\n             Win = %d\n", target.distance, absoluteX, window);

	if(absoluteX > window) // Needs changed.
		rotation = ((target.location.x + 25) / (width * 2.0f));// getRotation((float)target.location.x);// -640 to 640 aprox

	//VerticalDelta = Fuzzify((float)target.location.y, SmallDV, MediumDV, LargeDV);
	
	forward /= 4.0f;
	rotation *= 2.5f;
	float speed = getVerticalSpeed(); 

	ardrone.move3D(forward, 0.0f, speed, rotation);
	//fprintf(pFile, "- Move ----> Fwd = %3.2f, Rat = %3.2f, Dis = %3.2f, X = %d, Y = %d\n", forward, rotation, target.distance, target.location.x, target.location.y);
	fprint("- Move ----> Dis = %3.2f\n             Fwd = %3.2f\n             Rot = %3.2f\n             Spd = %3.2f\n             X = %d\n             Y = %d\n", target.distance, forward, rotation, speed, target.location.x, target.location.y);
}


// ------------------------------------------------------------------------------------
// Tracking::followTarget()
// Using the target information, the ARDrone is in follow mode and should maintain 
// height and position over target.
// ------------------------------------------------------------------------------------
void Tracking::followTarget(int j)
{
	// Theortically finished
	// control over 0-7 make 5 center;

	// 0 ft - 92
	// 5 ft - 80
	// 20 ft - 19
	/*
	float DistanceErrorOffset = 2.5f;
	float distance =  target.distance - DistanceErrorOffset;
	*/
	int x = target.location.x;
	
	float SmallError = 25.0f;
	float MediumError = 200.0f;
	float LargeError = width * 1.0f;

	float Left = 0.0f;
	float Right = 0.0f;

	if (x >= 0)
	{
		Left = Fuzzify((float)x, SmallError, MediumError, LargeError);
	}
	else
	{
		Right = Fuzzify((float)-x, SmallError, MediumError, LargeError);
	}
	
	float StrafeLR =  Right - Left;

	// 0 ft - 92
	// 5 ft - 80

	float FRMovement;

	if(target.distance > 0.0f)
	{
		float forward = 0.0f;
		float backward = 0.0f;
	
		float SmallDistance = 0.5f, MediumDistance = 1.0f, LargeDistance = 1.5f;
		float distance = target.distance - 8.5f;

		if(distance >= 0.0f)
		{
			forward = FuzzifyDistanceApproach(distance, SmallDistance, MediumDistance, LargeDistance);
		}
		else
		{
			backward = FuzzifyDistanceApproach(-distance, SmallDistance, MediumDistance, LargeDistance);
		}

		FRMovement = forward - backward;
	}
	else
	{

		FRMovement = -5.0f;
	}

	if (j < 5)
	{
		FRMovement = -0.2;
	}
	
	FRMovement /= 6.0f;
	StrafeLR /= 4.0f;
	float speed = getVerticalSpeed();

	//ardrone.move3D(FRMovement, StrafeLR, speed, 0.0f);
	// Testing the introduction of rotation into the follow method.
	ardrone.move3D(FRMovement, StrafeLR, speed, (-StrafeLR / 4.0f));
	
	if(!ardrone.onGround())
	{
		//fprintf(pFile, "- Follow --> Fwd = %3.2f, Str = %3.2f, Dis = %3.2f, X = %d, Y = %d\n", FRMovement, StrafeLR, target.distance, target.location.x, target.location.y);
		fprint("- Follow --> Fwd = %3.2f\n             Str = %3.2f\n             Dis = %3.2f\n             Spd = %3.2f\n             X = %d\n             Y = %d\n", FRMovement, StrafeLR, target.distance, speed, target.location.x, target.location.y);
	}
	// TODO :: Do some rotation, altitude adjustment, strife, and/or forward/backward motion
}

// ------------------------------------------------------------------------------------
// Tracking::rotate()
// No angle is supplied, so called the rotate method that takes an angle with this as a
// default angle of rotation.
// ------------------------------------------------------------------------------------
void Tracking::rotate()
{
	rotate(1.0f); // No parameter default to angle of 1. Whatever that means.
}

// ------------------------------------------------------------------------------------
// Tracking::swoop()
// This function manuevers the drone 180 degrees in orientation from its original position while moving
// to the other side of the person wearing the hat while still not ramming the person. 
// ------------------------------------------------------------------------------------
void Tracking::swoop()
{
	for (int i = 0; i < 1000; i++)
	{
		if (i % 100)
		{
			ardrone.move3D(-0.5f, 0.0f, 1.0f, 0.0f);
		}
	}
	for (int i = 0; i < 10000; i++)
	{
		if (i % 100)
		{
			ardrone.move3D(0.5f, -1.0f, 0.0f, -1.0f);
		}
	}

	// probably will have to significantly modify this.

}

// ------------------------------------------------------------------------------------
// Tracking::rotate(double angle)
// Angle is the amount of rotation that should take place.
// ------------------------------------------------------------------------------------
void Tracking::rotate(float angle)
{
	// TODO :: Rotate some angle
	//ardrone_at_set_progress_cmd
	// Assumption positive value is rotate right negative is rotate left. (if not execute next line)
	float rotation;

	if (angle != 0.0f)
	{
		if (angle > 0.0f)
		{
			rotation = 0.7f; // Rotation speed in Rot/Sec in the positive direction.
		}
		else 
		{
			rotation = -0.7f;
		}
		
		ardrone.move2D(0,0,rotation);
	}
	else 
	{
		ardrone.move2D(0,0,0);
	}

}