#include <stdlib.h>
#include <time.h>
#include <windows.h>
#include <stdio.h>

#include "ardrone\ardrone.h"
#include "project\project.h"
#include "project\helpers.h"

char *Space = "                                        ";
char *Line = "........................................";
int SpaceLength = strlen(Space);
int LineLength = strlen(Line) - 3;
IplImage *source;
double pointAveragerDistanceThreshold = 30.0f,
			 pointAveragerXYThreshold = 200;

float x_vel = 0,
	  	y_vel = 0,
		  z_vel = 0,
	  	r_vel = 0; 

int	pointAveragerSize = 1,
		
		prec_vel = 3,

		minHue= 23, 
		maxHue= 68,

		minSat= 0,     
		maxSat= 255,

		minLum= 0,         
		maxLum= 255,
	
		minBlob = 0,
		maxBlob = 10,

		m_thresh = 70,
		MA_tresh = 255,
		
		size_val=40,    
		size_center_val=20,

		factor_val=10,  
		factor_center_val=5,

		cenDis = 1,
		hiThresh = 100,
		loThresh = 50,
		minRad = 0,
		maxRad = 150;

int j = 0;

float PsiOnGround;
bool testingMode = false;

FILE * pFile;
// --------------------------------------------------------------------------
// main(Number of arguments, Value of arguments)
// This is the main function.
// Return value Success:0 Error:-1
// --------------------------------------------------------------------------
int main(int argc, char **argv)
{
	//relative path to project root folder
	pFile = fopen("..\\..\\log.txt", "w");
	if (pFile == NULL){
		printf("error regarding log.txt.");
	}
	
  /*
	char runNumber [100];

	puts ("Please Enter Run Number: ");
  gets (runNumber);
	*/
	fprintf(pFile, "/********************************\n");
	fprintf(pFile, " * Run Index:  \n");//, runNumber);
	fprintf(pFile, " ********************************/\n\n");
	
	Tracking tracking;		
	bool noWindow = true;

	// Main loop
	while (!GetAsyncKeyState(VK_ESCAPE))
	{
		home();

		// Update your AR.Drone
		if (!tracking.ardrone.update()) break;
				
		keypress(tracking);

		// Get an image
		IplImage *image = 0;

		// Find target and get Location
		if (false || tracking.getCurrentState() & 0x10)
		{
			// use front cam
			tracking.ardrone.setCamera(0);
			image = tracking.ardrone.getImage();
			tracking.findTargetUsingForwardCamera(image);
		}
		else if(tracking.getCurrentState() & 0x20)
		{
			// use down cam
			tracking.ardrone.setCamera(tracking.isVersion20 ? 1 : 2);
			image = tracking.ardrone.getImage();
			tracking.findTargetUsingDownCamera(image);
		}
				
		if(noWindow && image != 0)
		{
			int width = (int)(image->width * 1.25), 
					height = (int)(image->height * 1.25);

			cvDestroyAllWindows();

			cvCreateWindow("Camera", 150, 50, width, height);
			cvCreateWindow("Target", 150, height + 100, width, height);
			//cvCreateWindow("Controls", 250 + width, 50, 250, 500);

			//cvSetupTrackbar();

			noWindow = false;
		}
				
		//printf("\n\n%d\n\n", 1.0f);
		//printf("Command (%3.2f, %3.2f, %3.2f, %3.2f)       \n", x_vel, y_vel, z_vel, r_vel);
		//printf("Actual  (%3.2f, %3.2f, %3.2f, %3.2f)       \n\n", tracking.movement.speed.vx, tracking.movement.speed.vy, tracking.movement.speed.vz, tracking.movement.speed.velocity);
		//puts("Distance ", fp);

		tracking.stateMachine();
		/*
		switch(tracking.getCurrentState())
	{
		case StateInitState:
			{
				if(!tracking.ardrone.onGround())
					tracking.setState(StateInitHover);

				break;
			}
		case StateInitHover:
			{
				if(j < 50)//0000)
					j++;
				else
					tracking.setState(StateSetAltitude);
				
				tracking.ardrone.move2D(0.0f, 0.0f, 0.0f);
				break;
			}
		case StateSetAltitude:
			{
				if (tracking.ardrone.getAltitude() >= 1.75f)
					tracking.setState(StateFCSearching);

				tracking.ardrone.move3D(0.0f, 0.0f, 0.9f, 0.0f);
				break;
			}
		case StateFCSearching:
			{
				if(tracking.target.distance <= 5)
					tracking.setState(StateFCFollowing);
				else if(tracking.target.distance <= 20)
					tracking.setState(StateFCMoving);

				tracking.ardrone.move2D(0.0f, 0.0f, -0.75f);
				break;
			}
		case StateFCMoving:
			{
				if(tracking.target.distance > 20)
					tracking.setState(StateFCSearching);
				else if(tracking.target.distance <= 5)
					tracking.setState(StateFCFollowing);
						
				tracking.doMovement();
				break;
			}
		case StateFCFollowing:
			{
				if(tracking.target.distance > 20)
					tracking.setState(StateFCSearching);
				else if(tracking.target.distance > 7)
					tracking.setState(StateFCMoving);

				tracking.followTarget();
				break;
			}
		}
		*/
		// Print variable statuses
		tracking.print();

		// Display the image
		if(image) cvShowImage("Camera", image);			
		if(source) cvShowImage("Target", source);

		cvWaitKey(1);
	}
	
	return 0;
}

void consolePrint(int leftPad, const char *label, const char *type, ...)
{
	va_list vl;
	va_start(vl, type);
	
	printf("%s", Space + SpaceLength - leftPad);
	printf(" %s ", label);
	printf("%s", Line + leftPad + strlen(label));
	if(strlen(type) > 0){
		if(type[0] == '%'){
			vprintf(" =  %03d [%%]", vl);
		}
		else if(type[0] == 'T'){
			vprintf(" =  %s [T/F]", vl);
		}
		else{
			vprintf(" = % 3.2f", vl);
			printf(" [%s]", type);
		}	
	}
	else {
		vprintf(" = % 3.2f", vl);
	}
	printf("     \n");

	va_end(vl);
}

void home() {
	HANDLE hStdOut = GetStdHandle( STD_OUTPUT_HANDLE );
	//COORD pos = { 0, 0 };
	//SetConsoleCursorPosition( hStdOut, pos );
}

void voidFunction(int scale)
{
}

float velocityCalcs(int index)
{
	float range = 1.0f;
	return index < 1 ? -range : (index > 1 ? range : 0.0f);
	//double div = prec_vel / 2.0;
	//return (index - div) / div;
}

void velocityCalcsX(int index)
{
	x_vel = velocityCalcs(index);
}

void velocityCalcsY(int index)
{
	y_vel = velocityCalcs(index);
}

void velocityCalcsZ(int index)
{
	z_vel = velocityCalcs(index);
}

void velocityCalcsR(int index)
{
	r_vel = velocityCalcs(index);
}

void cvSetSize(int pos)
{
	if(size_val < 2) size_val = 2;
	size_center_val = (size_val / 2);
}

void cvSetFactor(int pos)
{
	if(factor_val < 2) factor_val = 2;
	factor_center_val = (factor_val / 2);
}

void cvErodeImage(IplImage *source, IplImage *destination)
{
	IplConvKernel *erode = cvCreateStructuringElementEx((int)((double)factor_val * 1.5), factor_val, factor_center_val, factor_center_val, CV_SHAPE_ELLIPSE);
	cvErode(source, destination, erode, 1);
}

void cvDilateImage(IplImage *source, IplImage *destination)
{
	IplConvKernel *dilate = cvCreateStructuringElementEx((int)((double)size_val * 1.5), size_val, size_center_val, size_center_val, CV_SHAPE_ELLIPSE);
	cvDilate(source, destination, dilate, 1);
}

void cvSetupTrackbar()
{
	cvNamedWindow("Controls",CV_WINDOW_NORMAL);
	
	//cvCreateTrackbar("Precision", "Controls", &prec_vel, prec_vel, voidFunction);
	
	int vel = prec_vel / 2;
	int xvel = vel;
	int yvel = vel;
	int zvel = vel;
	int rvel = vel;

	/*
	cvCreateTrackbar("X Velocity", "Controls", &xvel, 2, velocityCalcsX);
	cvCreateTrackbar("Y Velocity", "Controls", &yvel, 2, velocityCalcsY);
	cvCreateTrackbar("Z Velocity", "Controls", &zvel, 2, velocityCalcsZ);
	cvCreateTrackbar("R Velocity", "Controls", &rvel, 2, velocityCalcsR);

	cvCreateTrackbar("minHue", "Controls", &minHue, 256, voidFunction);
	cvCreateTrackbar("maxHue", "Controls", &maxHue, 256, voidFunction);

	cvCreateTrackbar("minSat", "Controls", &minSat, 256, voidFunction);
	cvCreateTrackbar("maxSat", "Controls", &maxSat, 256, voidFunction);

	cvCreateTrackbar("minLum", "Controls", &minLum, 256, voidFunction);
	cvCreateTrackbar("maxLum", "Controls", &maxLum, 256, voidFunction);

	cvCreateTrackbar("Dilate", "Controls", &size_val, 40, cvSetSize);  
	cvCreateTrackbar("Erode", "Controls", &factor_val, 40, cvSetFactor);
	
	cvCreateTrackbar("Threshold Min", "Controls", &m_thresh, 256, voidFunction);  
	cvCreateTrackbar("Threshold Max", "Controls", &MA_tresh, 256, voidFunction);
	*/
	
	cvCreateTrackbar("Center Distance", "Controls", &cenDis, 100, voidFunction);
	cvCreateTrackbar("High Thresh", "Controls", &hiThresh, 100, voidFunction);
	cvCreateTrackbar("Low Thresh", "Controls", &loThresh, 100, voidFunction);
	cvCreateTrackbar("Min Rad", "Controls", &minRad, 400, voidFunction);
	cvCreateTrackbar("Max Rad", "Controls", &maxRad, 400, voidFunction);
}
