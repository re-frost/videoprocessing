#ifndef OPENCVSLIDER_HPP
#define OPENCVSLIDER_HPP

#include <iostream>

#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/core/utility.hpp>

using namespace std;
using namespace cv;

class OpencvSlider
{
public:
    OpencvSlider(String);

    int test = 500;
    inline static String WINDOWNAME = "";
    inline static const int max_hue_value = 360/2;
    inline static const int max_V_S = 255;

    inline static int low_hue = 0;
    inline static int low_saturation = 0;
    inline static int low_value = 0;

    inline static int high_hue = max_hue_value;
    inline static int high_saturation = max_V_S;
    inline static int high_value = max_V_S;


    inline static bool selectRegion = false;
    inline static Rect selectedRect;
    inline static Point originPoint;
    inline static int trackingFlag = 0;
    inline static Mat image;
    static void onMouse(int, int, int, int, void*);

    static void on_low_H_thresh_trackbar(int, void *);
    static void on_high_H_thresh_trackbar(int, void *);
    static void on_low_S_thresh_trackbar(int, void *);
    static void on_high_S_thresh_trackbar(int, void *);
    static void on_low_V_thresh_trackbar(int, void *);
    static void on_high_V_thresh_trackbar(int, void *);

    void showSlider();
    String windowName;
    ~OpencvSlider();


};

#endif // OPENCVSLIDER_HPP
