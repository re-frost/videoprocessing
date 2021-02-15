#ifndef VIDEOPROCESSING_H
#define VIDEOPROCESSING_H

#include <string>
#include "show.h"
#include <opencv4/opencv2/videoio.hpp>

using namespace cv;
class VideoProcessing {

public:
    VideoProcessing(std::string = "");

    VideoCapture openCapture(std::string = "");
    ~VideoProcessing();
    void showBasicStream(std::string, char = 'U');

private:
    std::string filepath;
    VideoCapture capture;
};

#endif // VIDEOPROCESSING_H
