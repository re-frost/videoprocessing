#include "colortracking.hpp"

ColorTracking::ColorTracking(String winName) : OpencvSlider(winName)
{

    state = Mat(stateSize, 1, type);  // [x, y, v_x, v_y, w, h]
    meas = Mat(measSize, 1, type);    // [z_x, z_y, z_w, z_h]
    kf = KalmanFilter(stateSize, measSize, contrSize, type);
    //Mat procNoise(stateSize, 1, type)
    // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  0 0 ]
    // [ 0 0 0  1  0 0 ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    setIdentity(kf.transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    kf.measurementMatrix = Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    //setIdentity(kf.processNoiseCov, Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 10.0f;
    kf.processNoiseCov.at<float>(21) = 10.0f;
    kf.processNoiseCov.at<float>(28) = 1e-1;
    kf.processNoiseCov.at<float>(35) = 1e-1;

    // Measures Noise Covariance Matrix R
    setIdentity(kf.measurementNoiseCov, Scalar(1e-1));


    namedWindow("Debugger", WINDOW_AUTOSIZE);
}


Mat ColorTracking::colorTracking(Mat input) {

    setMouseCallback(windowName, onMouse, 0);

    // Define Ranges of channels
    float hueRanges[] = {0, 179};
    float saturationRanges[] = {0, 255};
    float valueRanges[] = {0, 255};

    const float* histHueRanges = hueRanges;
    const float* histSaturationRanges = saturationRanges;
    const float* histValueRanges = valueRanges;

    GaussianBlur(input, gaussianBlur, Size(5, 5), BORDER_REPLICATE);

    // Copy the image
    gaussianBlur.copyTo(image);

    // Convert to HSV colorspace
    cvtColor(image, hsvImage, COLOR_BGR2HSV);

    // Delta t -- Kalman filter
    double precTick = ticks;
    ticks = (double) getTickCount();
    double dT = (ticks - precTick) / getTickFrequency(); //seconds

    if(trackingFlag)
    {
        // Check for all the values in 'hsvimage' that are within the specified range
        // and put the result in 'mask'
        inRange(hsvImage, Scalar(lowHUE, minSaturation, minValue), Scalar(highHUE, maxSaturation, maxValue), mask);

        // Debugger
        cout << "Hue" << lowHUE << " " << highHUE << " " << test << endl;
        cout << "Saturation " << minSaturation << " " << maxSaturation << " " << test << endl;
        cout << "Value " << minValue << " " << maxValue << " " << test << endl;

        bitwise_and(input, input, frame_threshold, mask);
        imshow("Debugger", frame_threshold);


        if(trackingFlag < 0)
        {
            // When choose new region, reset all channels
            lowHUE = 0, highHUE = 179;
            minSaturation = 0, maxSaturation = 255;
            minValue = 0, maxValue = 255;
            // check in Range
            inRange(hsvImage, Scalar(lowHUE, minSaturation, minValue), Scalar(highHUE, maxSaturation, maxValue), mask);

            // Splt HSV Channels for histogram
            ColorTracking::prepareChannels();

            // Calculate Histogram & set boundries for mask
            ColorTracking::defineColorRange(histHueRanges, histSaturationRanges, histValueRanges);

            trackingRect = selectedRect;
            trackingFlag = 1;
        }

        if (found) {
            // >>>> Matrix A
            kf.transitionMatrix.at<float>(2) = dT;
            kf.transitionMatrix.at<float>(9) = dT;
            // <<<< Matrix A

            cout << "dT:" << endl << dT << endl;

            state = kf.predict();
            cout << "State post:" << endl << state << endl;
            cout << "Tracking rect " << endl << trackingRect << endl;
            cv::Rect predRect;
            predRect.width = state.at<float>(4);
            predRect.height = state.at<float>(5);
            predRect.x = state.at<float>(0) - predRect.width / 2;
            predRect.y = state.at<float>(1) - predRect.height / 2;

            cv::Point center;
            center.x = state.at<float>(0);
            center.y = state.at<float>(1);

            cv::circle(image, center, 2, Scalar(0,255,0), -1);
            cv::rectangle(image, predRect, Scalar(0,255,0), 2);
        }


        // Compute the histogram back projection
        calcBackProject(&hueImage, 1, 0, histHue, backproj, &histHueRanges);
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
        //        ellipse(image, rotatedTrackingRect, Scalar(0,255,0), 3, LINE_AA);
        // Draw rect
        rectangle(image, trackingRect, Scalar(255, 0, 0), 1, LINE_8, 0.5);

        // [z_x,z_y,z_w,z_h]
        meas.at<float>(0) = trackingRect.x + trackingRect.width / 2;
        meas.at<float>(1) = trackingRect.y + trackingRect.height / 2;
        meas.at<float>(2) = (float)trackingRect.width;
        meas.at<float>(3) = (float)trackingRect.height;

        if (!found) { // First detection!

            // >>>> Initialization
            kf.errorCovPre.at<float>(0) = 1; // px
            kf.errorCovPre.at<float>(7) = 1; // px
            kf.errorCovPre.at<float>(14) = 0.3;
            kf.errorCovPre.at<float>(21) = 0.3;
            kf.errorCovPre.at<float>(28) = 1; // px
            kf.errorCovPre.at<float>(35) = 1; // px

            state.at<float>(0) = meas.at<float>(0);
            state.at<float>(1) = meas.at<float>(1);
            state.at<float>(2) = 0;
            state.at<float>(3) = 0;
            state.at<float>(4) = meas.at<float>(2);
            state.at<float>(5) = meas.at<float>(3);
            // <<<< Initialization

            kf.statePost = state;

            found = true;
        }else
            kf.correct(meas); // Kalman Correction

        //        cout << "Measure matrix:" << endl << meas << endl;
    }

    // Apply the 'negative' effect on the selected region of interest
    if(selectRegion && selectedRect.width > 0 && selectedRect.height > 0)
    {
        Mat roi(image, selectedRect);
        bitwise_not(roi, roi);
    }

    return image;
}

void ColorTracking::defineColorRange(const float* histHueRanges,
                                     const float* histSaturationRanges,
                                     const float* histValueRanges) {

    // Create images based on selected regions of interest
    Mat roiHue(hueImage, selectedRect), roiSaturation(saturationImage, selectedRect);
    Mat roiVaue(valueImage, selectedRect), maskroi(mask, selectedRect);

    // Compute the histogram and normalize it
    calcHist(&roiHue, 1, 0, maskroi, histHue, 1, &histHueSize, &histHueRanges);
    calcHist(&roiSaturation, 1, 0, maskroi, histSaturation, 1, &histSaturationSize, &histSaturationRanges);
    calcHist(&roiVaue, 1, 0, maskroi, histvalue, 1, &histValueSize, &histValueRanges);

    double minH = 0, maxH = 179, minSV = 0, maxSV = 255;
    Point minLoc, maxLoc;
    minMaxLoc(histHue, &minH, &maxH, &minLoc, &maxLoc);
    lowHUE = (int)maxLoc.y - 3;
    highHUE = (int)maxLoc.y + 3;

    minMaxLoc(histSaturation, &minSV, &maxSV, &minLoc, &maxLoc);
    minSaturation = (int)maxLoc.y - 30;
    maxSaturation = (int)maxLoc.y + 30;

    minMaxLoc(histvalue, &minSV, &maxSV, &minLoc, &maxLoc);
    minValue = (int)maxLoc.y - 30;
    maxValue = (int)maxLoc.y + 30;

    normalize(histHue, histHue, 0, 180, NORM_MINMAX);
    normalize(histSaturation, histSaturation, 0, 255, NORM_MINMAX);
    normalize(histvalue, histvalue, 0, 255, NORM_MINMAX);
}

void ColorTracking::prepareChannels() {
    // Mix the specified channels
    // https://titanwolf.org/Network/Articles/Article?AID=ba3d9546-918b-4a52-920f-a6af746a1058#gsc.tab=0
    int channelsHue[] = {0, 0};
    int channelsSaturation[] = {1, 0};
    int channelsValue[] = {2, 0};

    hueImage.create(hsvImage.size(), hsvImage.depth());
    saturationImage.create(hsvImage.size(), hsvImage.depth());
    valueImage.create(hsvImage.size(), hsvImage.depth());

    mixChannels(&hsvImage, 1, &hueImage, 1, channelsHue, 1);
    mixChannels(&hsvImage, 1, &saturationImage, 1, channelsSaturation, 1);
    mixChannels(&hsvImage, 1, &valueImage, 1, channelsValue, 1);
}


ColorTracking::~ColorTracking(){}

