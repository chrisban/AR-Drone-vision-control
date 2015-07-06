#include <sys\utime.h>

#include "project.h"

extern int pointAveragerSize, j;
extern double pointAveragerDistanceThreshold, pointAveragerXYThreshold;
extern bool testingMode;
extern FILE *pFile;

DWORD loopTime;
DWORD startTime;
DWORD maxTime;
DWORD minTime;

Tracking::Tracking(){
	// Initialize
  if (!ardrone.open()) {
      printf("Failed to initialize.\n");
			system("pause");
      exit(-1);
  }

	//int seq = ardrone.getSequenceNumber();
	//ardrone.sockCommand.sendf("AT*CONFIG=%d,\"video:video_codec\",\"131\"\r", seq);
        
	//ardrone.resetEmergency();

	isVersion20 = ardrone.getVersion() == ARDRONE_VERSION_2;
	isSearching = true;
	testingMode = true;
	changeState = false;
	_currentState = 0x00;
	width = 320;
	height = 180;
	
	startTime = GetTickCount();
	loopTime = 0;
	maxTime = 0;
	minTime = -1;

	averager.clear();
	setTarget(0, 0, 0.0f, 0.0);
	target.isFound = false;
	
	updateMovement();

	fprint("\nTRACKING OBJECT RESET\n\n\n");
};

Tracking::~Tracking(){
	ardrone.close();
};

void Location::print() {
	consolePrint(0, "target.distance", "px²", distance * 1.0f);
	consolePrint(6, ".direction", "°", direction * 1.0f);
	consolePrint(6, ".location.x", "px", location.x);
	consolePrint(15, ".y", "px", location.y);
};

void Movement::print() {
	// Orientation
	consolePrint(0, "ardrone.orientation.roll", "°", orientation.roll * RAD_TO_DEG);
	consolePrint(19, ".pitch", "°", orientation.pitch * RAD_TO_DEG);
	consolePrint(19, ".yaw", "°", orientation.yaw * RAD_TO_DEG);
				
  // Altitude
	consolePrint(7, ".altitude", "m", altitude * 1.0f);
				
  // Velocity
	consolePrint(7, ".speed.velocity", "m/s", speed.velocity * 1.0f);
	consolePrint(13, ".vx", "m/s", speed.vx * 1.0f);
	consolePrint(13, ".vy", "m/s", speed.vy * 1.0f);
	consolePrint(13, ".vz", "m/s", speed.vz * 1.0f);

	// Battery
	consolePrint(7, ".battery", "%%", battery);
};

void Tracking::print() {
	return;
	DWORD endTime = GetTickCount();
	loopTime = endTime - startTime;
	
	if(loopTime > maxTime) maxTime = loopTime;
	if(loopTime < minTime) minTime = loopTime;
	
	updateMovement();

	printf("Object Data: %d < %d < %d [ms]\n\n", minTime, loopTime, maxTime);
	movement.print();
	consolePrint(0, "tracking.isSearching", "T/F", isSearching ? "True" : "False");
	target.print();
	
	printf("\n\nPoint Averager:\n");
	averager.print();

	startTime = endTime;
};

CvPoint3D64f Tracking::setTarget(int x, int y, float distance, double direction)
{	
	CvPoint3D64f pointDistance = averager.XYZ(x - width, height - y, distance);

	target.location.x = (int)pointDistance.x;
	target.location.y = (int)pointDistance.y;
	target.distance = (float)pointDistance.z;
	target.direction = direction;

	return pointDistance;
}

void Tracking::updateMovement() {
	movement.orientation.roll  = ardrone.getRoll();
  movement.orientation.pitch = ardrone.getPitch();
  movement.orientation.yaw   = ardrone.getYaw();
	
	movement.altitude = ardrone.getAltitude();
	
	movement.speed.velocity = ardrone.getVelocity(&movement.speed.vx, &movement.speed.vy, &movement.speed.vz);
	
	movement.battery = ardrone.getBatteryPercentage();
};

void keypress(Tracking &tracking) {
  // Take off / Landing
  //*
 	if (KEY_PUSH(VK_SPACE)) {
    if (tracking.ardrone.onGround())
		{
			j = 0;
			tracking.ardrone.takeoff();
			tracking.setState(StateInitState);
			tracking.ardrone.move3D(0.0f, 0.0f, 0.0f, 0.0f);
		}
		else
		{
			tracking.ardrone.move3D(0.0f, 0.0f, 0.0f, 0.0f);
			tracking.ardrone.landing();
			//CurrentState = StateLanding;
			tracking.setState(StateInitState);
			tracking.averager.clear();
			tracking.setTarget(0, 0, -1, 0.0);
		}
  }
	//*/

  // Emergency stop
  if (KEY_PUSH(VK_RETURN))
	{
		tracking.ardrone.emergency();
		tracking.ardrone.resetEmergency();
	}
  // AR.Drone is flying
	/*
	if (!tracking.ardrone.onGround()) {
      // Move
      float x = 0.0f, y = 0.0f, z = 0.0f, r = 0.0f, gain = 5.0f;

      if (KEY_DOWN(VK_UP))    y =  0.5f;
      if (KEY_DOWN(VK_DOWN))  y = -0.5f;
      if (KEY_DOWN(VK_LEFT))  r =  0.5f;
      if (KEY_DOWN(VK_RIGHT)) r = -0.5f;
      if (KEY_DOWN('Q'))      z =  0.5f;
      if (KEY_DOWN('A'))       z = -0.5f;

      tracking.ardrone.move3D(x * gain, y * gain, z * gain, r * gain);
  }
	*/

	// Change Camera
	if (KEY_PUSH('C')) {
		//tracking.isSearching = !tracking.isSearching;
		//printf("--> Change State Command Sent\n");
		//tracking.changeState = true;
		//tracking.averager.clear();
		tracking.setState(StateInitState);
	}
};

int Tracking::getCurrentState() 
{
	return _currentState;
}

void Tracking::setState(int newState)
{
  //if(_lastState != newState)//!testingMode || changeState)
	{
		_lastState = _currentState;
		_currentState = newState;

		//printf("Last State Is ................... 0x%x\n", _lastState);
		//printf("New State Is .................... 0x%x\n", _currentState);
		//fprintf(pFile, "Current State Is ................ 0x%x\n", _currentState);
		//printf("Current State Is ................ 0x%x\n", _currentState);
		fprint("Current State Is ................ 0x%x\n", _currentState);
		j = 0;
	}
}

double maxZ = 0.0;
double minZ = 9999999999.9;

CvPoint3D64f PointAverager::XYZ(int xx, int yy, float zz)
{
	if(zz < -8.0f || zz > 20.0f)
	{
		if(size == 0)
		{
			CvPoint3D64f point;

			point.x = 0.0;
			point.y = 0.0;
			point.z = -99999.0;

			return point;
		}
		
		return lastAverage;
	}

	if(size > 0)
	{
		double thresholdMax;
		double thresholdMin;
		
		thresholdMax = lastAverage.x + pointAveragerXYThreshold;
		thresholdMin = lastAverage.x - pointAveragerXYThreshold;

		if((xx < (int)thresholdMin) || (xx > (int)thresholdMax))
		{
			xx = (int)lastAverage.x;
		}
		
		thresholdMax = lastAverage.y + pointAveragerXYThreshold;
		thresholdMin = lastAverage.y - pointAveragerXYThreshold;

		if((yy < (int)thresholdMin) || (yy > (int)thresholdMax))
		{
			yy = (int)lastAverage.y;
		}
		
		thresholdMax = lastAverage.z + pointAveragerDistanceThreshold;
		thresholdMin = lastAverage.z - pointAveragerDistanceThreshold;

		if((zz < thresholdMin) || (zz > thresholdMax))
		{
			zz = (float)lastAverage.z;
		}
	}

	// 110 represents  3 feet
	//  20 represents 20 feet 0 - 90
	//zz = (zz / 67.0f) * 20.0f;
	/*if(zz > 110.0f)
		zz = -1.0f;
	else if(zz < 20.0f)
		zz = -1.0f;
	else
		zz = ((110.0f - zz) / 90.f) * 20.0f;

	CvPoint3D64f point;
	point.x = xx;
	point.y = yy;
	point.z = zz;
	return point;
	*/

	x[index] = xx;
	y[index] = yy;
	z[index] = zz;

	if(index < pointAveragerSize)
		index++;
	else
		index = 0;

	if(size < pointAveragerSize)
		size++;

	int rx = 0, ry = 0, i;
	double rz = 0.0;

	for(i = 0; i < size; i++)
	{
		double lz = z[i];

		rx += x[i];
		ry += y[i];
		rz += lz;

		if(lz < minZ)
			minZ = lz;

		if(lz > maxZ)
			maxZ = lz;
	}

	CvPoint3D64f point;
	if(i > 0)
	{
		double j = i * 1.0;

		point.x = rx / j;
		point.y = ry / j;
		point.z = rz / j;
	}
	else
	{
		point.x = 0.0;
		point.y = 0.0;
	}
	
	//fprint("- Averager > Max Z = %3.2f\n             Min Z = %3.2f\n", maxZ*1.0f, minZ*1.0f);

	point.z = zz;
	
	lastAverage = point;
	return point;
}

void PointAverager::clear()
{
	index = 0;
	size = 0;

	lastAverage.x = 0.0;
	lastAverage.y = 0.0;
	lastAverage.z = 0.0;
};

void PointAverager::print()
{
	return;
	double xx = 0.0, yy = 0.0, zz = 0.0;

	int i;
	for(i = 0; i < pointAveragerSize; i++)
	{
		xx += x[i];
		yy += y[i];
		zz += z[i];

		if(i < size)
			printf("\n   %2d     x = %6.2f     y = %6.2f     r=%6.2f     ", i + 1, x[i], y[i], z[i]);
		else
			printf("\n                                                   ");
	}
	
	printf("\n   Sum    x = %6.2f     y = %6.2f     r=%6.2f     ", xx, yy, zz);
	printf("\n   Avg    x = %6.2f     y = %6.2f     r=%6.2f     ", xx / i, yy / i, zz / i);
}

void fprint(const char *format, ...)
{
	va_list vl;
	va_start(vl, format);
	
	vfprintf(pFile, format, vl);
	vprintf(format, vl);

	va_end(vl);
}