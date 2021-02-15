#ifndef FRAMEPROCESSING_H
#define FRAMEPROCESSING_H

#include <string>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/video/background_segm.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/videoio.hpp>

using namespace cv;
using namespace std;

class FrameProcessing{
public:
    FrameProcessing();
    Mat toHSV(Mat);
    Mat autoCanny(Mat, int, int);
    Mat findContoursBasic(Mat, int);
    Mat backgroundSubtraction(Mat, string);



    Mat removeBackground(Mat, int, int);
    Mat removeLight(Mat, Mat, int);
    ~FrameProcessing();

private:
    Ptr<BackgroundSubtractor> pBackSub = createBackgroundSubtractorMOG2(5000, 20, false);
};

#endif // FRAMEPROCESSING_H
