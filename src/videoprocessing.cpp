#include "videoprocessing.h"
#include "staticUtils.h"


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
//    show.basicStream(capture, HSV_SLIDER, "HSV");
//    show.basicStream(capture, EDGE_SLIDER, "edge");
//    show.basicStream(capture, BACKGROUNDREMOVE_SLIDER, "Remove Background");

    show.basicStream(capture, "Background Subtraction", "Background Subtraction");
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

// void VideoProcessing::showBasicStream(std::string windowName, std::string modus){
//     Show show;
//     show.basicStream(capture, (String)windowName, modus);
// }