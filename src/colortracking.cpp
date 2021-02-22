#include "colortracking.hpp"

ColorTracking::ColorTracking( String winName) : windowName(winName)
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
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;

    // Measures Noise Covariance Matrix R
    setIdentity(kf.measurementNoiseCov, Scalar(1e-1));

    // Camera Index
    int idx = 0;
}

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

    GaussianBlur(input, gaussianBlur, Size(5, 5), BORDER_REPLICATE);

    // Clone the input frame
    gaussianBlur.copyTo(image);


    // Convert to HSV colorspace
    cvtColor(image, hsvImage, COLOR_BGR2HSV);

    // Delta t
    double precTick = ticks;
    ticks = (double) getTickCount();
    double dT = (ticks - precTick) / getTickFrequency(); //seconds

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

        if (found) {
            // >>>> Matrix A
            kf.transitionMatrix.at<float>(2) = dT;
            kf.transitionMatrix.at<float>(9) = dT;
            // <<<< Matrix A

            cout << "dT:" << endl << dT << endl;

            state = kf.predict();
            cout << "State post:" << endl << state << endl;

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
        //        ellipse(image, rotatedTrackingRect, Scalar(0,255,0), 3, LINE_AA);
        // Draw rect
        rectangle(image, trackingRect, Scalar(255, 0, 0), 1, LINE_8, 0.5);

        // [z_x,z_y,z_w,z_h]
        meas.at<float>(0) = trackingRect.x + trackingRect.width / 2;
        meas.at<float>(1) = trackingRect.y + trackingRect.height / 2;
        meas.at<float>(2) = (float)trackingRect.width;
        meas.at<float>(3) = (float)trackingRect.height;

        cout << kf.errorCovPre.size << endl;
        if (!found) { // First detection!

            // >>>> Initialization
            kf.errorCovPre.at<float>(0) = 1; // px
            kf.errorCovPre.at<float>(7) = 1; // px
            kf.errorCovPre.at<float>(14) = 1;
            kf.errorCovPre.at<float>(21) = 1;
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

        cout << "Measure matrix:" << endl << meas << endl;
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

