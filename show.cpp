#include "show.h"

#include <string>
#include <iostream>

#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/core/utility.hpp>

#include "frameprocessing.h"
#include "staticUtils.h"

using namespace cv;
Show::Show(){}

Show::~Show(){}

void Show::basicStream(VideoCapture &capture, String winName, char modus) {

    Mat frame, result, hsv, edges;

    namedWindow(winName, WINDOW_AUTOSIZE);
    resizeWindow(winName, 800, 800);

    FrameProcessing modframe;
    while(true) {
        int wk = cv::waitKey(100);
        capture >> frame;

        if (!frame.empty()) {

            if (modus == 'U'){

                result = frame;
            }else if (modus == 'H'){

                hsv = modframe.toHSV(frame);
                result = Show::hsvSlider(hsv);
            }else if (modus == 'E'){

                edges = modframe.autoCanny(frame, lower, upper);
                Show::edgeSlider(edges);
                result = edges;
            }else if (modus == 'R'){

                // Contour
                hsv = modframe.toHSV(frame);

                result = Show::hsvSlider(hsv);
            }else if (modus == 'B'){
                result = modframe.removeBackground(frame);
            }

            imshow(winName, result);

        }else if (frame.empty()){
            // Autorepeat
            capture.set(CAP_PROP_POS_FRAMES, 1);
        }
        if ((char)wk == 27)
            break;
    }
    cv::destroyAllWindows();
    capture.release();
}

void Show::edgeSlider(Mat frame) {

    Mat frame_threshold;
    createTrackbar("Low  edge", EDGE_SLIDER, &lower, upper_threshold, on_low_edge_thresh_trackbar);
    createTrackbar("upper  edge", EDGE_SLIDER, &upper, upper_threshold, on_heigh_edge_thresh_trackbar);
    cv::inRange(frame, Scalar(lower), Scalar(upper), frame_threshold);
}

Mat Show::hsvSlider(Mat frame) {

    Mat frame_threshold;
    createTrackbar("Low H", HSV_SLIDER, &low_H, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High H", HSV_SLIDER, &high_H, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low S", HSV_SLIDER, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", HSV_SLIDER, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", HSV_SLIDER, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", HSV_SLIDER, &high_V, max_value, on_high_V_thresh_trackbar);
    cv::inRange(frame, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
    return frame_threshold;
}

