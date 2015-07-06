//#include <math.h>

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
//    Y- <-----+-----> Y+
//             |(0,0)
//             |
//            \/X-
extern int m_thresh, MA_tresh, cenDis, hiThresh, loThresh, minRad, maxRad;
extern IplImage *source;
extern FILE *pFile;

float scaleFactor = 100000.0f / (39.4f - 5.6f);
//float expFactor = scaleFactor * scaleFactor;

void Tracking::cvSetTargetCenter(IplImage *source, IplImage *destination)
{
	CvMoments moments;
	cvMoments(source, &moments);

	double area = cvGetCentralMoment(&moments, 0, 0);
	double distance = -5.54775 * log(0.000000106878 * area);

	target.isFound = distance > -8.0 && distance < 20.0;

	CvPoint point;
	point.x = (int)(cvGetSpatialMoment(&moments, 1, 0) / area);
	point.y = (int)(cvGetSpatialMoment(&moments, 0, 1) / area);

	CvPoint3D64f setPoint = setTarget(point.x, point.y, (float)distance, -1);

	fprint("- Moments -> Dis = %3.2f\n", area);
	fprint("                   %3.2f\n", distance);
	fprint("                   %3.2f\n", setPoint.z);
	fprint("             X = %d\n", point.x);
	fprint("                 %3.2f\n", setPoint.x);
	fprint("             Y = %d\n", point.y);
	fprint("                 %3.2f\n", setPoint.y);

	CvPoint newPoint = cvPoint(width + (int)setPoint.x, height - (int)setPoint.y);

	cvCircle(destination,										// draw on the original image
		point,		// center point of circle
		8,													// 3 pixel radius of circle
		CV_RGB(0, 255, 0),									// draw pure green
		CV_FILLED);										// thickness, fill in the circle

	// draw a small green circle at center of detected object
	cvCircle(destination,										// draw on the original image
		newPoint,		// center point of circle
		4,													// 3 pixel radius of circle
		CV_RGB(255, 0, 0),									// draw pure green
		CV_FILLED);										// thickness, fill in the circle

	return;
	/*
	CvMemStorage* p_strStorage;			// necessary storage variable to pass into cvHoughCircles()

	CvSeq* p_seqCircles;				// pointer to an OpenCV sequence, will be returned by cvHough Circles() and will contain all circles
	// call cvGetSeqElem(p_seqCircles, i) will return a 3 element array of the ith circle (see next variable)

	float* p_fltXYRadius;				// pointer to a 3 element array of floats
	// [0] => x position of detected object
	// [1] => y position of detected object
	// [2] => radius of detected object

	p_strStorage = cvCreateMemStorage(0);	// allocate necessary memory storage variable to pass into cvHoughCircles()

	p_seqCircles = cvHoughCircles(source,		// input image, nothe that this has to be grayscale (no color)
	p_strStorage,			// provide function with memory storage, makes function return a pointer to a CvSeq
	CV_HOUGH_GRADIENT,	// two-pass algorithm for detecting circles, this is the only choice available
	6,					//2 size of image / 2 = "accumulator resolution", i.e. accum = res = size of image / 2
	source->height / cenDis,	// min distance in pixels between the centers of the detected circles
	hiThresh,						//100 high threshold of Canny edge detector, called by cvHoughCircles
	loThresh,						//50 low threshold of Canny edge detector, called by cvHoughCircles
	minRad,						//10 min circle radius, in pixels
	maxRad);						//400 max circle radius, in pixels

	IplImage* imgLocalDestination = cvCloneImage(destination);

	int count = 0;
	if(p_seqCircles->total == 0)
	{
	CvPoint3D64f setPoint = setTarget(0, 0, 0.0f, -1);
	averager.clear();
	}
	else
	{
	count = 1;
	//count = p_seqCircles->total;
	}

	for(int i=0; i < count; i++) // for each element in sequential circles structure (i.e. for each object detected)
	{
	// from the sequential structure, read the ith value into a pointer to a float
	p_fltXYRadius = (float*)cvGetSeqElem(p_seqCircles, i);

	CvPoint point = cvPoint((int)p_fltXYRadius[0], (int)p_fltXYRadius[1]);
	float radius = p_fltXYRadius[2];

	CvPoint3D64f setPoint = setTarget(point.x, point.y, radius, -1);

	CvPoint newPoint = cvPoint(width + (int)setPoint.x, height - (int)setPoint.y);

	//fprintf(pFile, "\n- Find ----> Dis = %3.2f, X = %d / %3.2f, Y = %d / %3.2f\n", setPoint.z, point.x, setPoint.x, point.y, setPoint.y);
	fprint("\n- Find ----> Dis = %3.2f\n                   %3.2f\n             X = %d\n                 %3.2f\n             Y = %d\n                 %3.2f\n", radius, setPoint.z, point.x, setPoint.x, point.y, setPoint.y);

	//printf("Hat Position x = %f, y = %f, r = %f \n", p_fltXYRadius[0],		// x position of center point of circle
	//												  p_fltXYRadius[1],	    // y position of center point of circle
	//												  p_fltXYRadius[2]);	// radius of circle

	cvCircle(imgLocalDestination,										// draw on the original image
	point,		// center point of circle
	8,													// 3 pixel radius of circle
	CV_RGB(0,255,0),									// draw pure green
	CV_FILLED);										// thickness, fill in the circle

	// draw a small green circle at center of detected object
	cvCircle(imgLocalDestination,										// draw on the original image
	newPoint,		// center point of circle
	4,													// 3 pixel radius of circle
	CV_RGB(255,0,0),									// draw pure green
	CV_FILLED);										// thickness, fill in the circle

	// draw a red circle around the detected object
	cvCircle(imgLocalDestination,										// draw on the original image
	newPoint,		// center point of circle
	(int) setPoint.z,							// radius of circle in pixels
	CV_RGB(0,0,255),									// draw pure red
	2);												// thickness of circle in pixels
	}	// end for

	cvReleaseImage(&destination);
	destination = cvCloneImage(imgLocalDestination);
	cvReleaseImage(&imgLocalDestination);
	*/
}

void Tracking::privateTargetFindingFunction(IplImage *image)
{
	// todo :: added generic target finding code here and set the target variables to be returned
	IplImage* redChannel = cvCreateImage(cvGetSize(image), 8, 1);
	IplImage* greenChannel = cvCreateImage(cvGetSize(image), 8, 1);
	IplImage* blueChannel = cvCreateImage(cvGetSize(image), 8, 1);

	cvSplit(image, blueChannel, greenChannel, redChannel, NULL);

	cvSub(greenChannel, blueChannel, blueChannel);
	cvSub(greenChannel, redChannel, redChannel);
	cvAdd(redChannel, blueChannel, greenChannel);
	//cvSub(greenChannel,redChannel,greenChannel);
	cvThreshold(greenChannel, greenChannel, 120, 180, CV_THRESH_BINARY);

	cvErodeImage(greenChannel, greenChannel);
	cvDilateImage(greenChannel, greenChannel);

	//Smooths target to make it easier to find edges.
	cvSmooth(greenChannel, greenChannel, CV_GAUSSIAN, 9, 9);

	cvReleaseImage(&source);
	source = cvCloneImage(greenChannel);
	cvSetTargetCenter(source, image);

	cvReleaseImage(&redChannel);
	cvReleaseImage(&greenChannel);
	cvReleaseImage(&blueChannel);
}

// ------------------------------------------------------------------------------------
// Tracking::findTargetUsingForwardCamera()
// Find the targets location on a video feed using the forward camera image
// Return value LOCATION with X, Y, and Distance
// Where X is the vertical offset of the target
//		 Y is the horizontal offset of the target
//		 Distance represents an estimation from 1 to 255 for how far away the object is 
//			or 0 when not found
// ------------------------------------------------------------------------------------
void Tracking::findTargetUsingForwardCamera(IplImage *image)
{
	// Process common target finding algorithm
	privateTargetFindingFunction(image);
}

// ------------------------------------------------------------------------------------
// Tracking::findTargetUsingDownCamera()
// Find the targets location on a video feed using the down camera image
// Return value LOCATION with X, Y, and Size
// Where X is the vertical offset of the target
//		 Y is the horizontal offset of the target
//		 Size represents an estimation from 1 to 255 for how far away the object is 
//			or 0 when not found
// ------------------------------------------------------------------------------------
void Tracking::findTargetUsingDownCamera(IplImage *image)
{
	// Process common target finding algorithm
	privateTargetFindingFunction(image);

	// TODO :: set the target.direction

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
}