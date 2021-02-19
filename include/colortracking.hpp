#ifndef COLORTRACKING_HPP
#define COLORTRACKING_HPP
#include <iostream>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/videoio.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/video/tracking.hpp>

using namespace cv;
using namespace std;


class ColorTracking
{
public:
    ColorTracking(String);


    inline static bool selectRegion = false;
    inline static Rect selectedRect;
    inline static Point originPoint;
    inline static int trackingFlag = 0;
    inline static Mat image;
    static void onMouse(int, int, int, int, void*);
    Mat colorTracking(Mat);


    ~ColorTracking();

private:
    String windowName;
    Mat hsvImage, hueImage, mask, hist, backproj;
    Rect trackingRect;
    int histSize = 256;
    float scalingFactor = 0.75;
    int minSaturation = 40;
    int minValue = 20;
    int maxValue = 245;
};

#endif // COLORTRACKING_HPP
