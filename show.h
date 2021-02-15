#ifndef SHOW_H
#define SHOW_H

#include <string>
#include <iostream>

#include <opencv4/opencv2/videoio.hpp>

#include "videoprocessing.h"
#include "frameprocessing.h"
#include "staticUtils.h"

using namespace cv;
class Show
{
public:
    Show();

    void showHSV();
    void basicStream(VideoCapture&, String, char);
    Mat hsvSlider(Mat);
    void edgeSlider(Mat);

    ~Show();

private:

};

#endif // SHOWVIDEO_H
