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
    void basicStream(VideoCapture&, String, std::string);
    Mat hsvSlider(Mat);
    void edgeSlider(Mat);
    void backgroundRemoveSlider();
    ~Show();

private:

};

#endif // SHOWVIDEO_H
