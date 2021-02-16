#ifndef SHOW_H
#define SHOW_H
#include <string>
#include <iostream>
#include <sstream>

#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/core/utility.hpp>

#include "videoprocessing.h"
#include "frameprocessing.h"
#include "staticUtils.h"

using namespace cv;
using namespace std;

class Show
{
public:
    Show();

    void showHSV();
    void basicStream(VideoCapture&, String, string);
    Mat hsvSlider(Mat);
    void edgeSlider();
    void backgroundRemoveSlider();
    ~Show();

private:
    Rect rect1;
    Rect rect2;
    Mat frame_threshold;
};

#endif // SHOWVIDEO_H
