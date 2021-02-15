#include <iostream>
#include <string>

#include "videoprocessing.h"

int main(int argc, char *argv[])
{
    std::string filepath;
    if (argc > 2)
        filepath = argv[1];
    else
        filepath = "";
    VideoProcessing Workshop(filepath);
    Workshop.showBasicStream("hsv", 'H');
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


//const int max_value_H = 360/2;
//const int max_value = 255;
//const String HSV_SLIDER = "HSV";
//int low_H = 0, low_S = 0, low_V = 0;
//int high_H = max_value_H, high_S = max_value, high_V = max_value;

//cv::Rect rect1, rect2;

//struct initRoi {
//    int x;
//    int y;
//}SelectRoi;

//static void on_low_H_thresh_trackbar(int, void *)
//{
//    low_H = min(high_H-1, low_H);
//    setTrackbarPos("Low H", HSV_SLIDER, low_H);
//}

//static void on_high_H_thresh_trackbar(int, void *)
//{
//    high_H = max(high_H, low_H+1);
//    setTrackbarPos("High H", HSV_SLIDER, high_H);
//}

//static void on_low_S_thresh_trackbar(int, void *)
//{
//    low_S = min(high_S-1, low_S);
//    setTrackbarPos("Low S", HSV_SLIDER, low_S);
//}

//static void on_high_S_thresh_trackbar(int, void *)
//{
//    high_S = max(high_S, low_S+1);
//    setTrackbarPos("High S", HSV_SLIDER, high_S);
//}

//static void on_low_V_thresh_trackbar(int, void *)
//{
//    low_V = min(high_V-1, low_V);
//    setTrackbarPos("Low V", HSV_SLIDER, low_V);
//}

//static void on_high_V_thresh_trackbar(int, void *)
//{
//    high_V = max(high_V, low_V+1);
//    setTrackbarPos("High V", HSV_SLIDER, high_V);
//}

//const String EDGE_SLIDER = "Edge Slide";
//int lower = 0;
//int upper = 100;

//static void on_low_edge_thresh_trackbar(int, void *)
//{
//    lower = min(upper-1, lower);
//    setTrackbarPos("Low threshold", EDGE_SLIDER, lower);
//}

//static void on_heigh_edge_thresh_trackbar(int, void *)
//{
//    upper = max(upper, lower+1);;
//    setTrackbarPos("Low threshold", EDGE_SLIDER, upper);
//}

//static void mouseCallBack(int event, int x, int y, int flags, void*){

//    if (event == cv::EVENT_LBUTTONDOWN) {
//        SelectRoi.x = y;
//        SelectRoi.y = x;
//        //        std::cout << "x-coord: " << x << " " << "y-coord: " << y << std::endl;

//    }
//}

//static Scalar randomColor( RNG& rng )
//{
//    auto icolor = (unsigned) rng;
//    return Scalar( icolor&255, (icolor>>8)&255, (icolor>>16)&255 );
//}

//cv::Mat removeLight(Mat img, Mat pattern, int method)
//{
//    cv::Mat aux;
//    // if method is normalization
//    if(method==1)
//    {
//        // Require change our image to 32 float for division
//        cv::Mat img32, pattern32;
//        img.convertTo(img32, CV_32F);
//        pattern.convertTo(pattern32, CV_32F);
//        // Divide the image by the pattern
//        aux= 1-(img32/pattern32);
//        // Convert 8 bits format and scale
//        aux.convertTo(aux, CV_8U, 255);
//    }else{
//        aux= pattern-img;
//    }
//    return aux;
//}

//double medianMat(cv::Mat Input)
//{
//    Input = Input.reshape(0,1);// spread Input Mat to single row
//    std::vector<double> vecFromMat;
//    Input.copyTo(vecFromMat); // Copy Input Mat to vector vecFromMat
//    std::nth_element(vecFromMat.begin(), vecFromMat.begin() + vecFromMat.size() / 2, vecFromMat.end());
//    return vecFromMat[vecFromMat.size() / 2];
//}

//cv::Mat FindContoursBasic(Mat img, int area)
//{
//    vector<vector<Point> > contours;
//    findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//    Mat output= Mat::zeros(img.rows,img.cols, CV_8UC3);
//    RNG rng(0xFFFFFFFF);
//    for(auto i=0; i<contours.size(); i++){
//        double Area = cv::contourArea(contours[i]);
//        if (Area < area) {
//            continue;
//        }
////                cv::Rect bb = cv::boundingRect(contours[i]);
////                cv::rectangle(output, bb, randomColor(rng));
//        //    std::cout << bb.x << " " << bb.y << " " << bb.height << " "<< bb.width << std::endl;
//        cv::drawContours(output, contours, i, randomColor(rng));

//    }
//    return output;
//}

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
