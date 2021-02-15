#include <iostream>

#include "frameprocessing.h"
#include "staticUtils.h"

using namespace cv;

FrameProcessing::FrameProcessing() {}

Mat FrameProcessing::FindContoursBasic(Mat img, int area)
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
//                cv::Rect bb = cv::boundingRect(contours[i]);
//                cv::rectangle(output, bb, randomColor(rng));
        //    std::cout << bb.x << " " << bb.y << " " << bb.height << " "<< bb.width << std::endl;
        cv::drawContours(output, contours, i, randomColor(rng));

    }
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

Mat FrameProcessing::removeBackground(Mat frame) {
        Mat gaussBlure, grayscale;
        cvtColor(frame, grayscale, cv::COLOR_RGB2GRAY);
        GaussianBlur(grayscale, gaussBlure, cv::Size(31, 31), cv::BORDER_CONSTANT);
        Mat noLight = FrameProcessing::removeLight(grayscale, gaussBlure, 1);

        // Binarize image for segment
        Mat img_thr;
        cv::threshold(noLight, img_thr, 40, 150, cv::THRESH_BINARY);
        return img_thr;
}

Mat FrameProcessing::toHSV(Mat input) {

    Mat hsvImage, gaussBlure;
    GaussianBlur(input, gaussBlure, Size(5, 5), BORDER_CONSTANT);
    cvtColor(gaussBlure, hsvImage, COLOR_BGR2HSV);
    return hsvImage;
}

Mat FrameProcessing::autoCanny(Mat input, int lower, int upper) {

    cv::Mat grayscale, gaussBlure, edges;
    cv::cvtColor(input, grayscale, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(grayscale, gaussBlure, cv::Size(7, 7), cv::BORDER_REPLICATE);

    cv::Canny(gaussBlure, edges, lower, upper, 3);

    return edges;
}

FrameProcessing::~FrameProcessing(){}
