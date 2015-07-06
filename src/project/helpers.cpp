#include "helpers.h"

void cvCreateWindow(const char *name, int top, int left, int width, int height)
{	
	cvNamedWindow(name, CV_WINDOW_KEEPRATIO);
	cvMoveWindow(name, top, left);	
	cvResizeWindow(name, width, height);
}