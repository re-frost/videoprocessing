#include "show.hpp"

using namespace cv;
Show::Show(){}

Show::~Show(){}
void Show::basicStream(VideoCapture &capture, String winName, string modus) {

    Mat frame, result, hsv, edges;

    rect1.x = 894; rect1.y = 233; rect1.height = 67; rect1.width = 393;
    rect2.x = 654; rect2.y = 114; rect2.height = 83; rect2.width = 162;

    namedWindow(winName, WINDOW_AUTOSIZE);

    FrameProcessing modframe;
    ColorTracking colortrack(winName);

    if (modus == "Edge") {
        Show::edgeSlider();
    }else if (modus == "Remove Background") {
        Show::backgroundRemoveSlider();
    }else if (modus == "HSV Slider") {
        Show::hsvSlider();
    }else if (modus == "Color Tracking"){
        colortrack.showSlider();
    }

    bool pausePressed = false;
    while(true) {
        int wk = cv::waitKey(250);
        if((char)wk == 'p'){
            pausePressed = !pausePressed;
        }else if(!((char)wk == 'p') && !pausePressed){
            capture >> frame;
        }
            
        if (!frame.empty()) {

            if (modus == "unchangned"){

                result = frame;
            }else if (modus == "HSV Slider"){

                result = modframe.hsvFilter(frame, low_H, high_H, low_S, high_S, low_V, high_V);
            }else if (modus == "Edge"){

                result = modframe.autoCanny(frame, lower, upper, min_kernel);
            }else if (modus == "Background Subtraction"){

                stringstream ss;
                ss << capture.get(CAP_PROP_POS_FRAMES);
                string frameNumberString = ss.str();

                result = modframe.backgroundSubtraction(frame, frameNumberString);
            }else if (modus == "Remove Background"){

                result = modframe.removeBackground(frame, low_br, high_br, low_blure, min_kernel);
            }else if(modus == "Color Tracking"){

                result = colortrack.colorTracking(frame);
            }

//            cv::rectangle(result, rect1, Scalar( 255, 0, 0 ), 3);
//            cv::rectangle(result, rect2, Scalar( 255, 0, 0 ), 3);
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

// Umbauen!!
// https://www.ccoderun.ca/programming/doxygen/opencv/tutorial_erosion_dilatation.html
void Show::backgroundRemoveSlider(){
    createTrackbar("Low threshold", BACKGROUNDREMOVE_SLIDER, &low_br, max_value_BR, on_low_backgroundRemove_thresh_trackbar);
    createTrackbar("upper threshold", BACKGROUNDREMOVE_SLIDER, &high_br, max_value_BR, on_heigh_backgroundRemove_thresh_trackbar);
    createTrackbar("blure threshold", BACKGROUNDREMOVE_SLIDER, &low_blure, max_blure, backgroundRemove_blure_thresh_trackbar);
    createTrackbar( "Kernel size:\n 2n +1", BACKGROUNDREMOVE_SLIDER, &min_kernel, max_kernel_size, backgroundRemove_kernel_size_trackbar);
}

void Show::edgeSlider() {
    createTrackbar("Low threshold", EDGE_SLIDER, &lower, upper_threshold, on_low_edge_thresh_trackbar);
    createTrackbar("upper threshold", EDGE_SLIDER, &upper, upper_threshold, on_heigh_edge_thresh_trackbar);
    createTrackbar( "Kernel size:\n 2n +1", EDGE_SLIDER, &min_kernel, max_kernel_size, edge_dilate_kernel_size_trackbar);
}

void Show::hsvSlider() {
    createTrackbar("Low H", HSV_SLIDER, &low_H, max_value_H, on_low_H_thresh_trackbar);
    createTrackbar("High H", HSV_SLIDER, &high_H, max_value_H, on_high_H_thresh_trackbar);
    createTrackbar("Low S", HSV_SLIDER, &low_S, max_value, on_low_S_thresh_trackbar);
    createTrackbar("High S", HSV_SLIDER, &high_S, max_value, on_high_S_thresh_trackbar);
    createTrackbar("Low V", HSV_SLIDER, &low_V, max_value, on_low_V_thresh_trackbar);
    createTrackbar("High V", HSV_SLIDER, &high_V, max_value, on_high_V_thresh_trackbar);
}
