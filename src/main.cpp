#include <iostream>
#include <string>

#include <opencv4/opencv2/xfeatures2d.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/stereo.hpp>
#include <opencv4/opencv2/ximgproc/disparity_filter.hpp>

#include "videoprocessing.hpp"
#include "depthimage.hpp"


int main(int argc, char *argv[])
{
    std::string filepath;
    if (argc > 1)
        filepath = argv[1];
    else
        filepath = "";

    VideoProcessing Workshop(filepath);

    //    Workshop.showBasicStream("unchangned", "unchangned");
    //    Workshop.showBasicStream(HSV_SLIDER, "HSV Slider");
    //    Workshop.showBasicStream(EDGE_SLIDER, "Edge");
    //    Workshop.showBasicStream(BACKGROUNDREMOVE_SLIDER, BACKGROUNDREMOVE_SLIDER);
    //    Workshop.showBasicStream("Background Subtraction", "Background Subtraction");
    //    Workshop.showBasicStream("Color Tracking", "Color Tracking");

    string sterepImagedir = "/home/felix/Documents/stereo-pairs";
    string subfolder[] = {"/cones", "/teddy", "/tsukuba", "/venus"};
    DepthImage depthimage;
    string leftImage = "/imL.png", rightImage = "/imR.png";

    Mat leftImg, rightImg, disparity;
    leftImg = cv::imread(sterepImagedir + subfolder[1] + leftImage, IMREAD_COLOR);
    rightImg = cv::imread(sterepImagedir + subfolder[1] + rightImage, IMREAD_COLOR);

    cvtColor(leftImg, leftImg, COLOR_BGR2GRAY);
    cvtColor(rightImg, rightImg, COLOR_BGR2GRAY);

    namedWindow("left", WINDOW_AUTOSIZE);
    namedWindow("right", WINDOW_AUTOSIZE);
    imshow("left", leftImg);
    imshow("right", rightImg);
    waitKey(0);

    //    float scalingFactor = 0.5;
    //    resize(leftImg, leftImg, Size(), scalingFactor, scalingFactor, INTER_AREA);
    //    resize(rightImg, rightImg, Size(), scalingFactor, scalingFactor, INTER_AREA);

//    std::vector<cv::KeyPoint> keypoints1, keypoints2;
//    cv::Ptr<cv::Feature2D> ptrFeature2D = SiftFeatureDetector::create(5000);

//    // Keypoint detection
//    ptrFeature2D->detect(leftImg, keypoints1);
//    ptrFeature2D->detect(rightImg, keypoints2);


//    // Extract the descriptor
//    cv::Mat descriptors1;
//    cv::Mat descriptors2;

//    ptrFeature2D->compute(leftImg,keypoints1,descriptors1);
//    ptrFeature2D->compute(rightImg,keypoints2,descriptors2);

//    cv::BFMatcher matcher(cv::NORM_L2);

//    // Match the two image descriptors
//    std::vector<cv::DMatch> outputMatches;
//    matcher.match(descriptors1,descriptors2, outputMatches);

//    // Convert keypoints into Point2f
//    std::vector<cv::Point2f> points1, points2;
//    for (std::vector<cv::DMatch>::const_iterator it= outputMatches.begin(); it!= outputMatches.end(); ++it) {

//        // Get the position of left keypoints
//        points1.push_back(keypoints1[it->queryIdx].pt);
//        // Get the position of right keypoints
//        points2.push_back(keypoints2[it->trainIdx].pt);
//    }



//    std::vector<uchar> inliers(points1.size(),0);
//    cv::Mat fundamental= cv::findFundamentalMat(
//                points1,points2, // matching points
//                inliers,         // match status (inlier or outlier)
//                cv::FM_RANSAC,   // RANSAC method
//                1.0,        // distance to epipolar line
//                0.98);     // confidence probability


//    cout << fundamental << endl;


//    // Compute homographic rectification
//    cv::Mat h1, h2;
//    cv::stereoRectifyUncalibrated(points1, points2,
//                                  fundamental,
//                                  leftImg.size(), h1, h2);
//    // Rectify the images through warping
//    cv::Mat rectified1;
//    cv::warpPerspective(leftImg, rectified1, h1, leftImg.size());
//    cv::Mat rectified2;
//    cv::warpPerspective(rightImg, rectified2, h2, rightImg.size());

    int mindisp = 0;
    int numDisp = 128;
    int blocksize = 9;

    cv::Ptr<stereo::StereoBinarySGBM> stereoBinSGB = stereo::StereoBinarySGBM::create(mindisp, numDisp, blocksize);

    stereoBinSGB->setP1(100);
    stereoBinSGB->setP2(1000);
    stereoBinSGB->setMinDisparity(0);
    stereoBinSGB->setUniquenessRatio(5);
    stereoBinSGB->setSpeckleWindowSize(400);
    stereoBinSGB->setSpeckleRange(0);
    stereoBinSGB->setDisp12MaxDiff(1);
    stereoBinSGB->setBinaryKernelType(stereo::CV_MODIFIED_CENSUS_TRANSFORM);
    stereoBinSGB->setSpekleRemovalTechnique(stereo::CV_SPECKLE_REMOVAL_AVG_ALGORITHM);
    stereoBinSGB->setSubPixelInterpolationMethod(stereo::CV_SIMETRICV_INTERPOLATION);

    stereoBinSGB->compute(leftImg, rightImg, disparity);

    // Natürlich darft du es nicht convertieren
    Mat result;
    disparity.convertTo(result, CV_8UC1);

    namedWindow("Disparity", WINDOW_AUTOSIZE);
    imshow("Disparity", result);
    waitKey(0);

    cv::imwrite("/home/felix/disparity.jpg", disparity);

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

