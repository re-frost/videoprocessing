#ifndef STATICUTILS_H
#define STATICUTILS_H

#include <iostream>

#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/core/utility.hpp>



using namespace std;
using namespace cv;

static String HSV_SLIDER = "HSV";
const int max_value_H = 360/2;
const int max_value = 255;

static int low_H = 0, low_S = 0, low_V = 0;
static int high_H = max_value_H, high_S = max_value, high_V = max_value;

const String EDGE_SLIDER = "Edge";
const int upper_threshold = 100;
static int lower = 0;
static int upper = upper_threshold;

const String BACKGROUNDREMOVE_SLIDER = "Remove Background";
const int max_value_BR = 255;
const int max_blure = 51;
const int max_kernel_size = 21;

static int low_br = 0;
static int high_br = 255;

static int low_blure = 1;
static int high_blure = max_blure;

static int min_kernel = 0;
static int kernel_size = max_kernel_size;

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
    upper = max(upper, lower+1);
    setTrackbarPos("upper threshold", EDGE_SLIDER, upper);
}

static void edge_dilate_kernel_size_trackbar(int, void *)
{
    min_kernel = min(min_kernel, kernel_size);
    setTrackbarPos("Blure threshold", EDGE_SLIDER, min_kernel);
}

// Threshold Background REMOVE
static void on_low_backgroundRemove_thresh_trackbar(int, void *)
{
    low_br = min(high_br - 1, low_br);
    setTrackbarPos("Low threshold", BACKGROUNDREMOVE_SLIDER, low_br);
}

static void on_heigh_backgroundRemove_thresh_trackbar(int, void *)
{
    high_br = max(high_br, low_br+1);
    setTrackbarPos("upper threshold", BACKGROUNDREMOVE_SLIDER, high_br);
}

static void backgroundRemove_blure_thresh_trackbar(int, void *)
{
    low_blure = min(max_blure, low_blure);
    if ((low_blure % 2 ==0) && (low_blure < max_blure)) {
        low_blure++;
    }
    setTrackbarPos("Blure threshold", BACKGROUNDREMOVE_SLIDER, low_blure);
}

static void backgroundRemove_kernel_size_trackbar(int, void *)
{
    min_kernel = min(min_kernel, kernel_size);
    setTrackbarPos("Blure threshold", BACKGROUNDREMOVE_SLIDER, min_kernel);
}

static Scalar randomColor( RNG& rng )
{
    auto icolor = (unsigned) rng;
    return Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
}

#endif // STATICUTILS_H
