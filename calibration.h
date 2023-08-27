#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <QObject>
#include <QThread>
#include <QMainWindow>
#include <QString>
#include "qmessagebox.h"
#include "qserialport.h"
#include "yrc1000micro_com.h"
#include "qobject.h"
#include "detector.h"
#include "calibration.h"
#include <unistd.h>
#include <QApplication>
#include <QFileDialog>
#include <QtMath>
/*---OpenCV---*/

#include <opencv4/opencv2/core.hpp>
#include <opencv2/aruco/aruco_calib.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/cvv/cvv.hpp"
/*---Visp Library---*/
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/vision/vpHandEyeCalibration.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpXmlParserHomogeneousMatrix.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/vision/vpPose.h>

#define 	CV_CALIB_CB_ADAPTIVE_THRESH   1

#define 	CV_CALIB_CB_FAST_CHECK   8

#define 	CV_CALIB_CB_FILTER_QUADS   4

#define 	CV_CALIB_CB_NORMALIZE_IMAGE   2

class Calibration : public QObject
{
    Q_OBJECT
public:
    explicit Calibration(QObject *parent = nullptr);
public:
    void intrinCalibration();
    void arucoDetection(int camIndex);
    void getPose(Mat calibFrame);
    void getPose_RealSense();
    vector<vpHomogeneousMatrix> getcMo_vec();
    vpHomogeneousMatrix getOnecMo();
    vpHomogeneousMatrix getbTc();
    vector<vpHomogeneousMatrix> cMo_vec;

     bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs);
    //---Hand Eye Calibration Function---
    void handEyeCalibration(QVector<double> , vpHomogeneousMatrix);

public:
    bool poseFlag;
    bool calibrationFlag = false;

    cv::Mat getUIImage();
public:
    double tagSize = 0.05;
private:

//  ---Marco Define ---
    //-- meters
    float quad_decimate = 1.0;
    int nThreads = 6;
    string intrinsic_file = "";
    string camera_name = "";
    bool display_tag = false;
    int color_id = -1;
    unsigned int thickness = 2;
    bool z_aligned = false;
    bool align_frame = true;
//  ---Class ---
    YRC1000micro_com yrc1000micro_com;
//  ---Variable ---
    //---Variables---
    vpRzyxVector rV;
    vpTranslationVector tV;
    vpImage<unsigned char> I;
    //---Input---
    vector<vpHomogeneousMatrix> eTb_vec;
    vector<vpHomogeneousMatrix> cTo_vec;
//    vector<vpHomogeneousMatrix> cMo_vec;
    vpHomogeneousMatrix cMo;
    vpHomogeneousMatrix bTc;
    vpRotationMatrix rM;
//    vpRxyzVector rV
    //  ---- Delay -------
    unsigned int microsecond = 1000000;
    cv::Mat uiImage;
    double intriPara_Intel[9] = {383.556, 0 , 324.112, 0, 383.556, 237.877, 0 , 0 , 1};

    double intriPara[9] = { 620.8510, 0, 308.9262, 0, 621.9056, 207.7374, 0, 0 ,1 };
    double intriPara_FullHD[9] = { 2272.0, 0, 966.9678, 0, 2261.7, 398.6332, 0, 0 ,1 };

    cv::Vec<double, 5> distCoeffs = {-0.0554, 1.1593, 0, 0, 0};
    std::vector<double> distortionCoeffs = {-0.0554, 1.1593, 0, 0 , 0};
//  ---XML file
    vpXmlParserHomogeneousMatrix pXML;
    std::string name_M;
    std::string filename;

public:
    double cxAruco, cyAruco;
    double central_x, central_y;
    double focal_x, focal_y;
    double xAruco, yAruco;
    double k1, k2; //Radial Distortion.
    //---Change the intrinsic matrix.
    bool fullHDFlag;
};

#endif // CALIBRATION_H
