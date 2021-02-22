#include "opencvslider.hpp"

OpencvSlider::OpencvSlider(String winName) : windowName(winName)
{
    OpencvSlider::WINDOWNAME = windowName;
}
void OpencvSlider::on_low_H_thresh_trackbar(int, void *)
{
    low_hue = min(high_hue-1, low_hue);
    setTrackbarPos("Low Hue", WINDOWNAME, low_hue);
}

void OpencvSlider::on_high_H_thresh_trackbar(int, void *)
{
    high_hue = max(high_hue, low_hue+1);
    setTrackbarPos("High Hue", WINDOWNAME, high_hue);
}

void OpencvSlider::on_low_S_thresh_trackbar(int, void *)
{
    low_saturation = min(high_saturation-1, low_saturation);
    setTrackbarPos("Low Saturation", WINDOWNAME, low_saturation);
}

void OpencvSlider::on_high_S_thresh_trackbar(int, void *)
{
    high_saturation = max(high_saturation, low_saturation+1);
    setTrackbarPos("High Saturation", WINDOWNAME, high_saturation);
}

void OpencvSlider::on_low_V_thresh_trackbar(int, void *)
{
    low_value = min(high_value-1, low_value);
    setTrackbarPos("Low Value", WINDOWNAME, low_value);
}

void OpencvSlider::on_high_V_thresh_trackbar(int, void *)
{

    high_value = max(high_value, low_value+1);
    setTrackbarPos("High Value", WINDOWNAME, high_value);
}

void OpencvSlider::onMouse(int event, int x, int y, int, void*)
{
    if(selectRegion)
    {
        selectedRect.x = MIN(x, originPoint.x);
        selectedRect.y = MIN(y, originPoint.y);
        selectedRect.width = std::abs(x - originPoint.x);
        selectedRect.height = std::abs(y - originPoint.y);

        selectedRect &= Rect(0, 0, image.cols, image.rows);
    }

    switch(event)
    {
    case EVENT_LBUTTONDOWN:
        originPoint = Point(x,y);
        selectedRect = Rect(x,y,0,0);
        selectRegion = true;
        break;

    case EVENT_LBUTTONUP:
        selectRegion = false;
        if( selectedRect.width > 0 && selectedRect.height > 0 )
        {
            trackingFlag = -1;
        }
        break;
    }

    cout << "originPoint coord " << originPoint.x << " " << originPoint.y << endl;
    cout << "x, y " << x << " " << y << endl;
    cout << "selectedRect coord " << selectedRect.x << " " << selectedRect.y << endl;
    cout << "selectedRect dimension " << selectedRect.width << " " << selectedRect.height << endl;
}

void OpencvSlider::showSlider() {
    createTrackbar("Low Hue", WINDOWNAME, &low_hue, max_hue_value, OpencvSlider::on_low_H_thresh_trackbar);
    createTrackbar("High Hue", WINDOWNAME, &high_hue, max_hue_value, OpencvSlider::on_high_H_thresh_trackbar);
    createTrackbar("Low Saturation", WINDOWNAME, &low_saturation, max_V_S, OpencvSlider::on_low_S_thresh_trackbar);
    createTrackbar("High Saturation", WINDOWNAME, &high_saturation, max_V_S, OpencvSlider::on_high_S_thresh_trackbar);
    createTrackbar("Low Value", WINDOWNAME, &low_value, max_V_S, OpencvSlider::on_low_V_thresh_trackbar);
    createTrackbar("High Value", WINDOWNAME, &high_value, max_V_S, OpencvSlider::on_high_V_thresh_trackbar);
}
OpencvSlider::~OpencvSlider(){}
