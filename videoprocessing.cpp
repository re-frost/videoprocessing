#include "videoprocessing.h"
#include "show.h"
#include <string>
#include "staticUtils.h"
#include <opencv4/opencv2/videoio.hpp>

using namespace cv;

VideoProcessing::VideoProcessing(std::string videofile)
    // Class to maniputate Videos
    : filepath{videofile} {

    // Capture contains viodeostream
    capture = openCapture(videofile);

    // HSV with slider: HSV_SLIDER
    // Canny with slider: EDGE_SLIDER
    // Remove Background: BACKGROUNDREMOVE_SLIDER
    Show show;
    show.basicStream(capture, BACKGROUNDREMOVE_SLIDER, "Remove Background");


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


VideoProcessing::~VideoProcessing(){}
