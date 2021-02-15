#ifndef SHOW_H
#define SHOW_H
#include <string>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/videoio.hpp>

#include "videoprocessing.h"

using namespace cv;
using namespace std;

class Show
{
public:
    Show();

    void showHSV();
    void basicStream(VideoCapture&, String, string);
    Mat hsvSlider(Mat);
    void edgeSlider(Mat);
    void backgroundRemoveSlider(Mat);
    ~Show();

private:

};

#endif // SHOWVIDEO_H
