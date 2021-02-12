#ifndef SHOW_H
#define SHOW_H
#include <string>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/videoio.hpp>

#include "videoprocessing.h"

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
