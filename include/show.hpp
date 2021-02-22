#ifndef SHOW_H
#define SHOW_H

#include <iostream>
#include <sstream>
#include <string>

#include "frameprocessing.hpp"
#include "staticUtils.hpp"

#include "colortracking.hpp"

#include <opencv4/opencv2/highgui.hpp>
using namespace cv;
using namespace std;

class Show
{
public:
    Show();

    void showHSV();
    void basicStream(VideoCapture&, String, string);
    void colorTracking();
    void hsvSlider();
    void edgeSlider();
    void backgroundRemoveSlider();
    ~Show();

private:
    Rect rect1;
    Rect rect2;

};

#endif // SHOWVIDEO_H
