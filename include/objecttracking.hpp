#ifndef OBJECTTRACKING_H
#define OBJECTTRACKING_H

#include <iostream>
#include <vector>
#include <string>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/videoio.hpp>

#include "target.h"

using namespace cv;
using namespace std;

#define MIN_TARGET_SIZE	250

class ObjectTracking
{
public:
    ObjectTracking(string);
    void LoadSequence(string);
    void frameprocessing();
    void Detecttarget();
    void TrackTarget();
    ~ObjectTracking();

private:
    VideoCapture capture;
    string filename_;
    Mat m_mSrcFrame3;
    Mat m_mThreshold;


    Mat m_mSrcFrame;
    Mat m_mDiffGaussian;
    // preproceed frame


    vector<Target> target;	// Tracking targets
    vector<Target> tempTarget; // Detected targets in current frame

    int nFrame;
};

#endif // OBJECTTRACKING_H
