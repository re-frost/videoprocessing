#ifndef VIDEOPROCESSING_H
#define VIDEOPROCESSING_H

#include <string>

#include <opencv4/opencv2/videoio.hpp>

#include "show.h"

using namespace cv;
class VideoProcessing {

public:
    VideoProcessing(std::string);

    VideoCapture openCapture(std::string);
    ~VideoProcessing();
    // void showBasicStream(String, std::string = "unchanged");

private:
    std::string filepath;
    VideoCapture capture;
};

#endif // VIDEOPROCESSING_H
