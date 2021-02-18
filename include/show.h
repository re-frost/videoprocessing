#ifndef SHOW_H
#define SHOW_H
// #include <string> //in frameprocessing.h
#include <iostream>
#include <sstream>

// #include <opencv4/opencv2/highgui/highgui.hpp> //in staticUtils.h
// #include <opencv4/opencv2/core/core.hpp> //in frameprocessing.h
// #include <opencv4/opencv2/imgproc/imgproc.hpp> //in frameprocessing.h
// #include <opencv4/opencv2/core/utility.hpp> //in staticUtils.h
// #include "videoprocessing.h" //cyclic dependency between show.h and videoprocessing.h

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
    void hsvSlider();
    void edgeSlider();
    void backgroundRemoveSlider();
    ~Show();

private:
    Rect rect1;
    Rect rect2;
};

#endif // SHOWVIDEO_H
