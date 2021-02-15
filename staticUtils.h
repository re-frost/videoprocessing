#ifndef STATICUTILS_H
#define STATICUTILS_H
#include <string>

#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/core/utility.hpp>
#include <opencv4/opencv2/core/core.hpp>

#include "frameprocessing.h"
using namespace std;
using namespace cv;

static String HSV_SLIDER = "HSV";
const int max_value_H = 360/2;
const int max_value = 255;

static int low_H = 0, low_S = 0, low_V = 0;
static int high_H = max_value_H, high_S = max_value, high_V = max_value;

const String EDGE_SLIDER = "Edge Slide";
const int upper_threshold = 100;

static int lower = 0;
static int upper = upper_threshold;

const String BACKGROUNDREMOVE_SLIDER = "Remve Background";
const int max_value_BR = 255;
static int low_br = 0;
static int high_br = 255;

static void on_low_H_thresh_trackbar(int, void *)
{
    low_H = min(high_H-1, low_H);
    setTrackbarPos("Low H", HSV_SLIDER, low_H);
}

static void on_high_H_thresh_trackbar(int, void *)
{
    high_H = max(high_H, low_H+1);
    setTrackbarPos("High H", HSV_SLIDER, high_H);
}

static void on_low_S_thresh_trackbar(int, void *)
{
    low_S = min(high_S-1, low_S);
    setTrackbarPos("Low S", HSV_SLIDER, low_S);
}

static void on_high_S_thresh_trackbar(int, void *)
{
    high_S = max(high_S, low_S+1);
    setTrackbarPos("High S", HSV_SLIDER, high_S);
}

static void on_low_V_thresh_trackbar(int, void *)
{
    low_V = min(high_V-1, low_V);
    setTrackbarPos("Low V", HSV_SLIDER, low_V);
}

static void on_high_V_thresh_trackbar(int, void *)
{
    high_V = max(high_V, low_V+1);
    setTrackbarPos("High V", HSV_SLIDER, high_V);
}

// Threshold Auto Canny
static void on_low_edge_thresh_trackbar(int, void *)
{
    lower = min(upper-1, lower);
    setTrackbarPos("Low threshold", EDGE_SLIDER, lower);
}

static void on_heigh_edge_thresh_trackbar(int, void *)
{
    upper = max(upper, lower+1);;
    setTrackbarPos("upper threshold", EDGE_SLIDER, upper);
}

// Threshold Background REMOVE
static void on_low_backgroundRemove_thresh_trackbar(int, void *)
{
    lower = min(high_br - 1, low_br);
    setTrackbarPos("Low threshold", BACKGROUNDREMOVE_SLIDER, low_br);
}

static void on_heigh_backgroundRemove_thresh_trackbar(int, void *)
{
    upper = max(high_br, low_br+1);;
    setTrackbarPos("upper threshold", BACKGROUNDREMOVE_SLIDER, high_br);
}

static void mouseCallBack(int event, int x, int y, int flags, void*){

//    if (event == cv::EVENT_LBUTTONDOWN) {
//        SelectRoi.x = y;
//        SelectRoi.y = x;
//        //        std::cout << "x-coord: " << x << " " << "y-coord: " << y << std::endl;

//    }
}


static Scalar randomColor( RNG& rng )
{
    auto icolor = (unsigned) rng;
    return Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
}

#endif // STATICUTILS_H
