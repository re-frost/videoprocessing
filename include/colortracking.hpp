#ifndef COLORTRACKING_HPP
#define COLORTRACKING_HPP
#include <iostream>
#include <vector>

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



    Mat colorTracking(Mat);


    ~ColorTracking();

private:
    Mat hsvImage, hueImage, mask, hist, backproj, res;
    Rect trackingRect;
    int histSize = 256;
    float scalingFactor = 1;
    int minSaturation = 40;
    int minValue = 20;
    int maxValue = 245;

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
