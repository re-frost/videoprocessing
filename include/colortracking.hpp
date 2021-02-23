#ifndef COLORTRACKING_HPP
#define COLORTRACKING_HPP
#include <iostream>
#include <vector>
#include <typeinfo>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/video/tracking.hpp>
#include <opencv4/opencv2/video/video.hpp>

#include "opencvslider.hpp"

using namespace cv;
using namespace std;


class ColorTracking : public OpencvSlider {

public:
    ColorTracking(String);


    void prepareChannels();
    void defineColorRange(const float*, const float*, const float*);
    Mat colorTracking(Mat);


    ~ColorTracking();

private:
    Mat hsvImage, hueImage, saturationImage, valueImage;
    Mat mask, backproj, res;
    Mat histHue, histSaturation, histvalue;

    Rect trackingRect;
    int histHueSize = 179;
    int histSaturationSize = 255;
    int histValueSize = 255;

    int minSaturation = 0;
    int maxSaturation = 255;

    int minValue = 0;
    int maxValue = 255;

    int lowHUE = 0;
    int highHUE = 179;

    // >>>> Kalman Filter
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;
    unsigned int type = CV_32F;
    KalmanFilter kf;


    Mat image_32F, state, meas, gaussianBlur;
    char ch = 0;

    double ticks = 0;
    bool found = false;
    int notFoundCount = 0;

    // Debugger
    Mat frame_threshold;

};

#endif // COLORTRACKING_HPP
