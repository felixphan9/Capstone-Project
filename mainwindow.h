#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QString>
#include <QDebug>
#include "qmessagebox.h"
#include "qserialport.h"
#include "yrc1000micro_com.h"
#include "qobject.h"
#include "detector.h"
#include "calibration.h"
#include <QApplication>
#include <QFileDialog>
#include <QtMath>
#include <unistd.h>
#include <QFuture>
#include <QtConcurrent>
#include <delay_timer.h>
#include <QFile>
#include <QTextStream>
//----------OpenCV--------------------
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/dnn.hpp>
#include <opencv4/opencv2/dnn/dnn.hpp>
#include <opencv4/opencv2/imgproc.hpp>
//-----------Visp--------------
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/vision/vpHandEyeCalibration.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/vision/vpHandEyeCalibration.h>
#include <visp/vpHomogeneousMatrix.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
using namespace cv;
using namespace std;
using namespace cv::dnn;
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    //      Private function for converting pulses value -> degree
    QVector<double> convert_pulses_to_degs(QVector<double> &pulses);

private slots:

    //      Slots for update UI form
    void updateUICallback(bool, bool, bool);
    //      Increment decrement position button
    void inc_dec_btn_clicked();
    //      Timer call back function
    void timerCallback();
    void pulsesTimerCallback();

    //      Basic Control
    void on_btnConnect_clicked();

    void on_btnServoOn_clicked();

    void on_btnGetPos_clicked();

    void on_btnSetPos_clicked();

    void on_btnHome_clicked();

    void on_sliderDismm_sliderMoved(int position);

    void on_sliderDisdeg_sliderMoved(int position);

    void on_sliderSpeedmm_sliderMoved(int position);

    void on_sliderSpeeddeg_sliderMoved(int position);

//    void on_btnGetJoints_clicked();

//    void on_btnCalcFK_clicked();

    void on_btnSerOpen_clicked();

    void on_btnSerClose_clicked();

    void on_btnMotorRun_clicked();

    void on_btnSelectJob_clicked();

    void on_btnStartJob_clicked();

    void on_btnTeach_clicked();

    //  Computer Vision For Detection
    void updateFrameandGui();

    void on_btnYOLO_clicked();

    void on_btnCamOnAuto_clicked();

    void on_btnBrowseYOLO_clicked();

    void on_btnDetectIP_clicked();

    void on_btnBG_clicked();

    void on_btnFG_clicked();

    void on_btnTEST_clicked();

    // Calibration

    void updateFrameConfig();

    void calibTimerCallback();

    void on_btnGetPose_clicked();

    void on_btnCamOnConfig_clicked();

    void on_btnStartCalib_clicked();

    void on_btnForwardKinematics_clicked();

    void on_btnIntriCalib_clicked();

    void on_btnArucoDetection_clicked();

    void on_btnStartAuto_clicked();

    void on_btnSaveFile_clicked();

    void on_btnStartAutoIP_clicked();

    bool writeFile(QString);

    void on_btnCalculate_clicked();
    //================================================================

private:

    QMessageBox msgBox;
    QTimer *timer;
    QTimer *camTimer;
    QTimer *camConfigTimer;
    QTimer *calibTimer;
    QTimer *pulsesTimer;
    QTimer *dataRCVTimer;
    QThreadPool pool;
    QThread solderThread;

    Ui::MainWindow *ui;
    YRC1000micro_com yrc1000micro_com;
    QSerialPort *mSerial;

    //---Variables-------------
    QVector<double> robot_position;
    double homeX, homeY, homeZ, homeRx, homeRy, homeRz;
    bool feedFlag;

    //---Computer Vision-------
    Detector tracker;
    VideoCapture cap;
    vpRealSense2 g;
    rs2::config config;
    unsigned int width = 640, height = 480;
    float depth_scale;

    //---Frame and Images-------
    Mat frame;
    Mat img;
    QImage imdisplay;  //This will create QImage which is shown in Qt label
    vpImage<unsigned char> I;

    //---Calibration -----------------
    Calibration calibration;
    uint16_t count = 0; // Count the times
    int refCount;
    vector<vpHomogeneousMatrix> eTb_vec;
    vector<vpHomogeneousMatrix> cTo_vec;
    vpHomogeneousMatrix bTc;

    //---Delay -------
    unsigned int microsecond = 1000000;
    delay_timer* delay;

    //---Auto soldering system---
    double pixeltomm = 0.423333;
    double pixeltomm_1080 = 0.1016;
    vector<double> cxPad_vec, cyPad_vec;
    vector<double> xPad_vec, yPad_vec;
    vector<double> X_vec, Y_vec;
    double intriPara_FullHD[9] = { 2272.0, 0, 966.9678, 0, 2261.7, 398.6332, 0, 0 ,1 };

    //----Robot position for pad only X Y change----
    double Rz, Ry, Rx;
    double Z;
    double cxAruco;
    double cyAruco;
    double deltaX;
    double deltaY;
    double xAruco, yAruco;

    //---Check extrinsics matrix:
    vpHomogeneousMatrix cTo_check, eTb_check;

    //---Saving File
    vpXmlParserHomogeneousMatrix pXML;
    std::string name_M;
    std::string filename;
    //---Saving text file
    QString timeS;
public:
    bool pos_flag;
    bool read_pos;
    bool serFlag;
};

#endif // MAINWINDOW_H
