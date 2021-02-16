#include <iostream>
#include <string>

#include "videoprocessing.h"


int main(int argc, char *argv[])
{

//    if (videofile != ""){
//        cap.open(videofile, cv::CAP_ANY);
//    }else {
//        cap.open(0);
//    }
    std::string filepath = argv[1];

    VideoProcessing Workshop(filepath);

    std::cout << "Program finished normally" << std::endl;
    return 0;
}



/*
+--------+----+----+----+----+------+------+------+------+
|        | C1 | C2 | C3 | C4 | C(5) | C(6) | C(7) | C(8) |
+--------+----+----+----+----+------+------+------+------+
| CV_8U  |  0 |  8 | 16 | 24 |   32 |   40 |   48 |   56 |
| CV_8S  |  1 |  9 | 17 | 25 |   33 |   41 |   49 |   57 |
| CV_16U |  2 | 10 | 18 | 26 |   34 |   42 |   50 |   58 |
| CV_16S |  3 | 11 | 19 | 27 |   35 |   43 |   51 |   59 |
| CV_32S |  4 | 12 | 20 | 28 |   36 |   44 |   52 |   60 |
| CV_32F |  5 | 13 | 21 | 29 |   37 |   45 |   53 |   61 |
| CV_64F |  6 | 14 | 22 | 30 |   38 |   46 |   54 |   62 |
+--------+----+----+----+----+------+------+------+------+
*/


//cv::Mat modifyStream(cv::Mat frame, char modus) {


//    //        std::cout << frame.type() << std::endl;  // 16 >> cv_8U c:3
//    switch(modus) {
//    case 'h': {
//        Mat hsvImage, gaussBlure;
//        cv::GaussianBlur(frame, gaussBlure, cv::Size(5, 5), cv::BORDER_CONSTANT);
//        cv::cvtColor(gaussBlure, hsvImage, cv::COLOR_RGB2HSV);
//        return hsvImage;
//    }
//    case 'r':{
//        cv::Mat gaussBlure;
//        cv::cvtColor(frame, frame, cv::COLOR_RGB2GRAY);
//        cv::GaussianBlur(frame, gaussBlure, cv::Size(51, 51), cv::BORDER_CONSTANT);
//        cv::Mat noLight = removeLight(frame, gaussBlure, 1);

//        // Binarize image for segment
//        Mat img_thr;
//        cv::threshold(noLight, img_thr, 40, 255, cv::THRESH_BINARY);
//        return img_thr;
//    }
//    case 'e': {
//        cv::Mat gaussBlure, edges;
//        cv::cvtColor(frame, frame, cv::COLOR_RGB2GRAY);
//        cv::GaussianBlur(frame, gaussBlure, cv::Size(7, 7), cv::BORDER_REPLICATE);
//        cv::Canny(gaussBlure, edges, lower, upper, 3);
//        return edges;
//    }
//    case 't': {
//        cv::Mat grayscale, hsvFrame;
////        cvtColor(frame, hsvFrame, COLOR_BGR2HSV );
//        cvtColor(frame, grayscale, COLOR_BGR2GRAY );

//        vector<Point2f> corners;
//        double qualityThreshold = 0.01;
//        double minDist = 10;
//        int blockSize = 5;
//        bool useHarrisDetector = false;
//        double k = 0.1;
//        int numCorners = 12;
//        goodFeaturesToTrack(grayscale, corners, numCorners, qualityThreshold, minDist, Mat(), blockSize, useHarrisDetector, k);

//        // Parameters for the circles to display the corners
//        int radius = 8;      // radius of the cirles
//        int thickness = 2;   // thickness of the circles
//        int lineType = 8;

//        RNG rng(12345);
//        // Draw the detected corners using circles
//        for(size_t i = 0; i < corners.size(); i++){
//            Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
//            circle(frame, corners[i], radius, color, thickness, lineType, 0);
//        }
//        return frame;
//    }

//    case '1': {
//        Mat hsvImage, gaussBlure, grayscale, hsvFeaterTrack;
//        cv::GaussianBlur(frame, gaussBlure, cv::Size(5, 5), cv::BORDER_CONSTANT);
//        cv::cvtColor(gaussBlure, hsvImage, cv::COLOR_RGB2HSV);
//        vector<Mat> channels;
//        cv::split(hsvImage, channels);
//        cv::cvtColor(hsvImage, grayscale, COLOR_BGR2GRAY );

//        vector<Point2f> corners;
//        double qualityThreshold = 0.01;
//        double minDist = 10;
//        int blockSize = 5;
//        bool useHarrisDetector = false;
//        double k = 0.1;
//        int numCorners = 15;
//        goodFeaturesToTrack(channels[1], corners, numCorners, qualityThreshold, minDist, Mat(), blockSize, useHarrisDetector, k);

//        // Parameters for the circles to display the corners
//        int radius = 8;      // radius of the cirles
//        int thickness = 2;   // thickness of the circles
//        int lineType = 8;

//        RNG rng(12345);
//        // Draw the detected corners using circles
//        for(size_t i = 0; i < corners.size(); i++){
//            Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
//            circle(hsvImage, corners[i], radius, color, thickness, lineType, 0);
//        }
//        return hsvImage;
//    }

//    case '2': {
//        cv::Mat gaussBlure, edges, grayscale, hsvframe;

//        cv::cvtColor(frame, hsvframe, cv::COLOR_RGB2HSV);
//        vector<Mat> channels;
//        cv::split(hsvframe, channels);
//        cv::GaussianBlur(channels[1], gaussBlure, cv::Size(35, 35), cv::BORDER_REPLICATE);

//        cv::Canny(gaussBlure, edges, lower, upper, 5);

//        cv::Mat result = FindContoursBasic(edges, 200);

//        return result;
//    }
//    default: return frame;

//    }
//}

//void loadVideo(std::string videoFile) {
//    cv::VideoCapture cap;
//    if (videoFile != ""){
//        cap.open(videoFile, cv::CAP_ANY);
//    }else {
//        cap.open(0);
//    }

//    char modus = 'h';


//    // Trackbars to set thresholds for HSV values
//    if (modus == 'h'){
//        cv::namedWindow(HSV_SLIDER, cv::WINDOW_AUTOSIZE);
//        cv::namedWindow("Contours", cv::WINDOW_AUTOSIZE);
//        cv::namedWindow("result", cv::WINDOW_AUTOSIZE);
//        cv::namedWindow("Workshop", cv::WINDOW_AUTOSIZE);
//    }
//    if (modus == 'e' || modus == '2'){
//        cv::namedWindow(EDGE_SLIDER, cv::WINDOW_AUTOSIZE);
//    }else {
//        cv::namedWindow("Workshop", cv::WINDOW_AUTOSIZE);
//    }
//    while(true)
//    {
//        cv::Mat frame, frame_threshold, result;
//        cap >> frame; // get a new frame from camera

//        int wk = cv::waitKey(100);

//        if(!frame.empty()){

//            cv::Mat newframe = modifyStream(frame, modus);

//            if (modus == 'h') {
//                createTrackbar("Low H", HSV_SLIDER, &low_H, max_value_H, on_low_H_thresh_trackbar);
//                createTrackbar("High H", HSV_SLIDER, &high_H, max_value_H, on_high_H_thresh_trackbar);
//                createTrackbar("Low S", HSV_SLIDER, &low_S, max_value, on_low_S_thresh_trackbar);
//                createTrackbar("High S", HSV_SLIDER, &high_S, max_value, on_high_S_thresh_trackbar);
//                createTrackbar("Low V", HSV_SLIDER, &low_V, max_value, on_low_V_thresh_trackbar);
//                createTrackbar("High V", HSV_SLIDER, &high_V, max_value, on_high_V_thresh_trackbar);
//                cv::inRange(newframe, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);


//                cv::Mat contourFrame = FindContoursBasic(frame_threshold, 5000);


//                cv::imshow(HSV_SLIDER, frame_threshold);

//                cv::bitwise_and(newframe, newframe, result, frame_threshold);

//                cv::imshow(HSV_SLIDER, frame_threshold);
//                cv::imshow("Contours", contourFrame);
//                cv::imshow("Workshop", newframe);
//                cv::imshow("result", result);
//            }
//            if (modus == 'e' || modus == '2') {
//                createTrackbar("Low  edge", EDGE_SLIDER, &lower, 100, on_low_edge_thresh_trackbar);
//                createTrackbar("upper  edge", EDGE_SLIDER, &upper, 100, on_heigh_edge_thresh_trackbar);
//                cv::inRange(newframe, Scalar(lower), Scalar(upper), frame_threshold);

//                cv::imshow(EDGE_SLIDER, newframe);
//            }else{
//                //            rect1.x = 894; rect1.y = 233; rect1.height = 67; rect1.width = 393;
//                //            rect2.x = 654; rect2.y = 114; rect2.height = 83; rect2.width = 162;
//                //            cv::rectangle(newframe, rect1, Scalar( 255, 0, 0 ), 2);
//                //            cv::rectangle(newframe, rect2, Scalar( 255, 0, 0 ));
//                cv::imshow("Workshop", newframe);

//                //            cv::setMouseCallback("Workshop", mouseCallBack, 0);
//                //            std::cout << "x-coord: " << SelectRoi.x << " " << "y-coord: " << SelectRoi.y << std::endl;
//                //            cv::Vec3b pix = newframe.at<Vec3b>(SelectRoi.x, SelectRoi.y);
//                //            std::cout << "B: " << (uint)pix[0] << " " << "G: " << (uint)pix[1] << " " << "R: " << (uint)pix[2] << std::endl;
//                //            cv::Mat bgr = cv::Mat(1, 1, CV_8UC3, cv::Scalar(pix[0],pix[1], pix[2]));
//                //            cv::Mat hsv;
//                //            cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
//                //            cv::Vec3b hsv_pix = hsv.at<Vec3b>(0, 0);
//                //            std::cout << "H: " << (uint)hsv_pix[0] << " " << "S: " << (uint)hsv_pix[1] << " " << "V: " << (uint)hsv_pix[2] << std::endl;

//            }



//        } else if (frame.empty()){
//            cap.set(cv::CAP_PROP_POS_FRAMES, 1);
//        }

//        if ((char)wk == 27)
//            break;
//    }
//    cv::destroyAllWindows();
//    // Release the camera or video cap
//    cap.release();

//}
