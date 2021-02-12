#ifndef FRAMEPROCESSING_H
#define FRAMEPROCESSING_H

#include <opencv4/opencv2/core/core.hpp>

using namespace cv;

class FrameProcessing{
public:
    FrameProcessing();
    Mat toHSV(Mat);
    Mat autoCanny(Mat, int, int);
    Mat FindContoursBasic(Mat, int);

    Mat removeBackground(Mat);
    Mat removeLight(Mat, Mat, int);
    ~FrameProcessing();

};

#endif // FRAMEPROCESSING_H
