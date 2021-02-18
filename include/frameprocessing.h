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
    Mat autoCanny(Mat, int, int, int);
    Mat findContoursMat(Mat, int);

    Mat backgroundSubtraction(Mat, string);


    Mat dilatation(Mat, int, int);
    Mat removeBackground(Mat, int, int, int, int);
    Mat removeLight(Mat, Mat, int);
    Mat hsvFilter(Mat, int, int, int, int, int, int);
    ~FrameProcessing();

private:
    Ptr<BackgroundSubtractor> pBackSub = createBackgroundSubtractorMOG2(5000, 20, false);
    MorphShapes dilation_type;
};

#endif // FRAMEPROCESSING_H
