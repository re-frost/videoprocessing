#include <iostream>

#include "frameprocessing.hpp"
#include "staticUtils.hpp"

using namespace cv;

FrameProcessing::FrameProcessing() {}

Mat FrameProcessing::hsvFilter(Mat img, int low_H, int high_H, int low_S, int high_S, int low_V, int high_V) {
    Mat hsv = toHSV(img);
    Mat mask, frame_threshold;
    inRange(hsv, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), mask);
    bitwise_and(img, img, frame_threshold, mask);
    return frame_threshold;
}

Mat FrameProcessing::backgroundSubtraction(Mat input, string frameNumberString) {

    Mat output, fgMask, gaussBlure, grayscale;
    cv::cvtColor(input, grayscale, COLOR_BGR2GRAY);

    GaussianBlur(grayscale, gaussBlure, Size(5, 5), BORDER_CONSTANT);
    pBackSub->apply(gaussBlure, fgMask);

    //get the frame number and write it on the current frame
    rectangle(fgMask, cv::Point(10, 2), cv::Point(100,20),
              cv::Scalar(255,255,255), -1);

    putText(fgMask, frameNumberString.c_str(), cv::Point(15, 15),
            FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(0,0,0));

    bitwise_and(input, input, output, fgMask);

    return output;
}

Mat FrameProcessing::findContoursMat(Mat img, int area)
{
    vector<vector<Point> > contours;
    findContours(img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    Mat output= Mat::zeros(img.rows,img.cols, CV_8UC3);

    RNG rng(0xFFFFFFFF);
    for(auto i=0; i<contours.size(); i++){
        double Area = cv::contourArea(contours[i]);
        if (Area < area) {
            continue;
        }

//        cv::Rect bb = cv::boundingRect(contours[i]);
//        cv::rectangle(img, bb, randomColor(rng));
        cv::drawContours(img, contours, i, randomColor(rng), 5);


    }
    return img;
}


Mat FrameProcessing::autoCanny(Mat input, int lower, int upper, int kernel_s) {

    Mat grayscale, gaussBlure, edges;
    cvtColor(input, grayscale, cv::COLOR_BGR2GRAY);
    GaussianBlur(grayscale, gaussBlure, Size(15, 15), cv::BORDER_REPLICATE);

    Canny(gaussBlure, edges, lower, upper, 3);
    Mat output = dilatation(edges, 0, kernel_s);
//    Mat contours = findContoursMat(output, 100);
    return output;
}

Mat FrameProcessing::dilatation(Mat input, int type, int size) {

    Mat output;
    if( type == 0 ){ dilation_type = MORPH_RECT; }
    else if( type == 1 ){ dilation_type = MORPH_CROSS; }
    else if( type == 2) { dilation_type = MORPH_ELLIPSE; }

    Mat element = getStructuringElement( type,
                         Size( 2*size + 1, 2*size+1 ),
                         Point( size, size ) );

    dilate( input, output, element );

    return output;
}

Mat FrameProcessing::removeLight(Mat img, Mat pattern, int method){
    Mat aux;
    // if method is normalization
    if(method==1)
    {
        // Require change our image to 32 float for division
        Mat img32, pattern32;
        img.convertTo(img32, CV_32F);
        pattern.convertTo(pattern32, CV_32F);
        // Divide the image by the pattern
        aux= 1-(img32/pattern32);
        // Convert 8 bits format and scale
        aux.convertTo(aux, CV_8U, 255);
    }else{
        aux= pattern-img;
    }
    return aux;
}

Mat FrameProcessing::removeBackground(Mat input, int lower, int upper, int blure_Value, int kernel_s) {
    Mat gaussBlure, grayscale;
    cvtColor(input, grayscale, cv::COLOR_RGB2GRAY);
    GaussianBlur(grayscale, gaussBlure, cv::Size(blure_Value, blure_Value), cv::BORDER_CONSTANT);
    Mat noLight = FrameProcessing::removeLight(grayscale, gaussBlure, 0);

    // Binarize image for segment
    Mat img_thr;
    threshold(noLight, img_thr, lower, upper, cv::THRESH_BINARY);

    Mat dilate = dilatation(img_thr, 0, kernel_s);
    return dilate;
}

Mat FrameProcessing::toHSV(Mat input) {

    Mat hsvImage, gaussBlure;
    GaussianBlur(input, gaussBlure, Size(5, 5), BORDER_CONSTANT);
    cvtColor(gaussBlure, hsvImage, COLOR_BGR2HSV);
    return hsvImage;
}

FrameProcessing::~FrameProcessing(){}
