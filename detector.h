#ifndef DETECTOR_H
#define DETECTOR_H

#include <QObject>
#include <QDebug>
#include <opencv4/opencv2/cvconfig.h>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/dnn.hpp>
#include <opencv4/opencv2/dnn/dnn.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/tracking.hpp>
#include <fstream>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;
using namespace cv::dnn;

class Detector : public QObject
{
    Q_OBJECT
public:
    explicit Detector(QObject *parent = nullptr);

public:
    void draw_label(cv::Mat& input_image, std::string label, int left, int top);
    vector<cv::Mat> pre_process(cv::Mat &input_image, cv::dnn::Net &net);
    cv::Mat post_process(cv::Mat &input_image, vector<cv::Mat> &outputs, const vector<std::string> &class_name);
    void bgSubtraction(cv::Mat , cv::Mat);
    void blobDetection(cv::Mat bg_img);
    vector<double> getcxPadVec();
    vector<double> getcyPadVec();
    void clearPadvec();

private:
//      Constants.
    const float INPUT_WIDTH = 640.0;
    const float INPUT_HEIGHT = 640.0;
    const float SCORE_THRESHOLD = 0.5;
    const float NMS_THRESHOLD = 0.45;
    const float CONFIDENCE_THRESHOLD = 0.45;

//      Text parameters.
//    const float FONT_SCALE = 0.35;
    const float FONT_SCALE = 0.2;
    const int FONT_FACE = FONT_HERSHEY_SIMPLEX;
    const float THICKNESS = 0.5;

    // Colors.
    Scalar BLACK = Scalar(0,0,0);
    Scalar BLUE = Scalar(255, 178, 50);
    Scalar YELLOW = Scalar(0, 255, 255);
    Scalar RED = Scalar(0,0,255);

    //---Background substraction:
    /* 0: Binary
       1: Binary Inverted
       2: Threshold Truncated
       3: Threshold to Zero
       4: Threshold to Zero Inverted
    */
    int threshold_value = 40;
    int threshold_type = 0;
    int const max_value = 255;
    int const max_type = 4;
    int const max_BINARY_value = 255;

    int morph_elem = 0;
    int morph_size = 0;
    int morph_operator = 0;

private:
    vector<double> cxPad_vec;
    vector<double> cyPad_vec;

public:
    // Class lists.
    vector<string> class_list = {"pad"};
    vector<double> x_vec;
    vector<double> y_vec;

signals:

};

#endif // DETECTOR_H
