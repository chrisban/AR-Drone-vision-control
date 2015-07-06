#ifndef __HELPERS_HPP__
#define __HELPERS_HPP__

#include <opencv2/opencv.hpp>

void cvInRangeFilter(const char *window, IplImage *source, CvScalar *min_color, CvScalar *max_color);
void cvSetTargetCenter(const char *name, IplImage *source, IplImage *destination);
void cvSetRGB(int pos);
void cvSetSize(int pos);
void cvSetFactor(int pos);
void cvCreateWindow(const char *name, int top, int left, int width, int height);
void cvSetupTrackbar();
void cvDilateImage(const char *window, IplImage *source);
void cvErodeImage(const char *window, IplImage *source);

#endif