#include "videoprocessing.h"
#include "staticUtils.h"

using namespace cv;

VideoProcessing::VideoProcessing(std::string videofile)
    // Class to maniputate Videos
    : filepath{videofile} {

    capture = openCapture(videofile);


}

VideoCapture VideoProcessing::openCapture(std::string videofile) {

    VideoCapture cap;
    if (videofile != ""){
        cap.open(videofile, cv::CAP_ANY);
    }else {
        cap.open(0);
    }

    return cap;
}

void VideoProcessing::showBasicStream(std::string windowName, std::string modus){
    Show show;
    show.basicStream(capture, (String)windowName, modus);
}

VideoProcessing::~VideoProcessing(){}

