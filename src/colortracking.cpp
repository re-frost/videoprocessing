#include "colortracking.hpp"

ColorTracking::ColorTracking( String winName) : windowName(winName)
{}

void ColorTracking::onMouse(int event, int x, int y, int, void*)
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

Mat ColorTracking::colorTracking(Mat input) {

    setMouseCallback(windowName, onMouse, 0);

    float hueRanges[] = {0,180};
    const float* histRanges = hueRanges;

//    resize(input, input, Size(), scalingFactor, scalingFactor, INTER_AREA);

    // Clone the input frame
    input.copyTo(image);

    // Convert to HSV colorspace
    cvtColor(image, hsvImage, COLOR_BGR2HSV);

    if(trackingFlag)
    {
        // Check for all the values in 'hsvimage' that are within the specified range
        // and put the result in 'mask'
        inRange(hsvImage, Scalar(0, minSaturation, minValue), Scalar(180, 256, maxValue), mask);

        // Mix the specified channels
        int channels[] = {0, 0};
        hueImage.create(hsvImage.size(), hsvImage.depth());
        mixChannels(&hsvImage, 1, &hueImage, 1, channels, 1);

        if(trackingFlag < 0)
        {
            // Create images based on selected regions of interest
            Mat roi(hueImage, selectedRect), maskroi(mask, selectedRect);

            // Compute the histogram and normalize it
            calcHist(&roi, 1, 0, maskroi, hist, 1, &histSize, &histRanges);
            normalize(hist, hist, 0, 255, NORM_MINMAX);

            trackingRect = selectedRect;
            trackingFlag = 1;
        }

        // Compute the histogram back projection
        calcBackProject(&hueImage, 1, 0, hist, backproj, &histRanges);
        backproj &= mask;
        RotatedRect rotatedTrackingRect = CamShift(backproj, trackingRect, TermCriteria(TermCriteria::EPS | TermCriteria::COUNT, 10, 1));

        // Check if the area of trackingRect is too small
        if(trackingRect.area() <= 1)
        {
            // Use an offset value to make sure the trackingRect has a minimum size
            int cols = backproj.cols, rows = backproj.rows;
            int offset = MIN(rows, cols) + 1;
            trackingRect = Rect(trackingRect.x - offset, trackingRect.y - offset, trackingRect.x + offset, trackingRect.y + offset) & Rect(0, 0, cols, rows);
        }

        // Draw the ellipse on top of the image
        ellipse(image, rotatedTrackingRect, Scalar(0,255,0), 3, LINE_AA);
    }

    // Apply the 'negative' effect on the selected region of interest
    if(selectRegion && selectedRect.width > 0 && selectedRect.height > 0)
    {
        Mat roi(image, selectedRect);
        bitwise_not(roi, roi);
    }

    return image;
}

ColorTracking::~ColorTracking(){}

