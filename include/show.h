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
<<<<<<< HEAD
    void edgeSlider(Mat);
=======
    void edgeSlider();
>>>>>>> 225e92ee81cb1fc91d618da8cfa0547dab2b0872
    void backgroundRemoveSlider();
    ~Show();

private:
    Rect rect1;
    Rect rect2;
    Mat frame_threshold;
};

#endif // SHOWVIDEO_H
