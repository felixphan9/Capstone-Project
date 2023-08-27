#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QUdpSocket>
#include <QMessageBox>
#include <QTimer>
using namespace cv;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      ui(new Ui::MainWindow)
{
    this->setWindowIcon(QIcon ("/home/phuc/Pictures/logo_BK-removebg.png"));
    ui->setupUi(this);
    ui->btnConnect->setAutoFillBackground(true);
    ui->btnConnect->setPalette(QColorConstants::Svg::green);
//      connections between dataUIReceiveSignal <-> updateUICallback.
//      When user want to update UI form, just emit the dataUIReceiveSignal()

    mSerial = new QSerialPort();
    timer = new QTimer(this);
    pulsesTimer = new QTimer(this);
    calibTimer = new QTimer(this);
    dataRCVTimer = new QTimer(this);

    connect(timer,SIGNAL(timeout()),this,SLOT(timerCallback()));
    connect(pulsesTimer,SIGNAL(timeout()),this,SLOT(pulsesTimerCallback()));
    connect(calibTimer,SIGNAL(timeout()),this,SLOT(calibTimerCallback()));

    connect(&yrc1000micro_com, SIGNAL(dataUIReceiveSignal(bool,bool,bool)), this, SLOT(updateUICallback(bool,bool,bool)));

    //---Connect 12 signals click to 1 slot
    connect(ui->btnXminus, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnXplus, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnYminus, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnYplus, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnZminus, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnZplus, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnRminus, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnRplus, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnPminus, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnPplus, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnYawminus, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnYawplus, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    //---------------------------Manual--------------------------------------------------
    connect(ui->btnXminusMan, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnXplusMan, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnYminusMan, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnYplusMan, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnZminusMan, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnZplusMan, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnRminusMan, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnRplusMan, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnPminusMan, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnPplusMan, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnYawminusMan, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    connect(ui->btnYawplusMan, SIGNAL(clicked()), this, SLOT(inc_dec_btn_clicked()));
    //---Connect signals from YRC1000micro and slots on_MotorRun------------------------
    connect(&yrc1000micro_com, SIGNAL(motorRun()), this, SLOT(on_btnMotorRun_clicked()));
    //---Delay
    delay = new delay_timer();
    //---Time and Date
    QTime time = QTime::currentTime();
    timeS = time.toString();

    QString qsCount = ui->txtBoxCalibCount->text();
    refCount = qsCount.toInt();
    //---Prepare to solder home position;
    homeX = 5.200;
    homeY = -235.000;
    homeZ = 20.500;
    homeRx = 170.2550;
    homeRy = -36.9800;
    homeRz = -80.6517;
}

MainWindow::~MainWindow()
{
    cap.release();
    delete ui;
    destroyAllWindows();
}

void MainWindow::timerCallback()
{
    yrc1000micro_com.YRC1000microReadStatus();
    yrc1000micro_com.YRC1000microReadPosition();
}

void MainWindow::pulsesTimerCallback()
{
    yrc1000micro_com.YRC1000microReadPulse();
}

void MainWindow::inc_dec_btn_clicked()
{
     QPushButton *current_bt = (QPushButton*)sender();

     double set_speed;
     bool mode_selection = true;
     double distance_mm = (double)ui->sliderDismm->value();
     double distance_deg = (double)ui->sliderDisdeg->value();

     QVector<double> set_position = QVector<double> (6);

     if (current_bt->objectName() == "btnXplus"){
         set_position[0] = distance_mm;
         mode_selection=true;
     }else if(current_bt->objectName() == "btnYplus"){
         set_position[1] = distance_mm;
         mode_selection=true;
     }else if(current_bt->objectName() == "btnZplus"){
         set_position[2] = distance_mm;
         mode_selection=true;
     }else if(current_bt->objectName() == "btnRplus"){
         set_position[3] = distance_deg;
         mode_selection=false;
     }else if(current_bt->objectName() == "btnPplus"){
         set_position[4] = distance_deg;
         mode_selection=false;
     }else if(current_bt->objectName() == "btnYawplus"){
         set_position[5] = distance_deg;
         mode_selection=false;
     }else if(current_bt->objectName() == "btnXminus"){
         set_position[0] = -distance_mm;
         mode_selection=true;
     }else if(current_bt->objectName() == "btnYminus"){
         set_position[1] = -distance_mm;
         mode_selection=true;
     }else if(current_bt->objectName() == "btnZminus"){
         set_position[2] = -distance_mm;
         mode_selection=true;
     }else if(current_bt->objectName() == "btnRminus"){
         set_position[3] = -distance_deg;
         mode_selection=false;
     }else if(current_bt->objectName() == "btnPminus"){
         set_position[4] = -distance_deg;
         mode_selection=false;
     }else if(current_bt->objectName() == "btnYawminus"){
         set_position[5] = -distance_deg;
         mode_selection=false;
     }

     if (current_bt->objectName() == "btnXplusMan"){
         set_position[0] = distance_mm;
         mode_selection=true;
     }else if(current_bt->objectName() == "btnYplusMan"){
         set_position[1] = distance_mm;
         mode_selection=true;
     }else if(current_bt->objectName() == "btnZplusMan"){
         set_position[2] = distance_mm;
         mode_selection=true;
     }else if(current_bt->objectName() == "btnRplusMan"){
         set_position[3] = distance_deg;
         mode_selection=false;
     }else if(current_bt->objectName() == "btnPplusMan"){
         set_position[4] = distance_deg;
         mode_selection=false;
     }else if(current_bt->objectName() == "btnYawplusMan"){
         set_position[5] = distance_deg;
         mode_selection=false;
     }else if(current_bt->objectName() == "btnXminusMan"){
         set_position[0] = -distance_mm;
         mode_selection=true;
     }else if(current_bt->objectName() == "btnYminusMan"){
         set_position[1] = -distance_mm;
         mode_selection=true;
     }else if(current_bt->objectName() == "btnZminusMan"){
         set_position[2] = -distance_mm;
         mode_selection=true;
     }else if(current_bt->objectName() == "btnRminusMan"){
         set_position[3] = -distance_deg;
         mode_selection=false;
     }else if(current_bt->objectName() == "btnPminusMan"){
         set_position[4] = -distance_deg;
         mode_selection=false;
     }else if(current_bt->objectName() == "btnYawminusMan"){
         set_position[5] = -distance_deg;
         mode_selection=false;
     }


     set_speed = (mode_selection==true)?(double)ui->sliderSpeedmm->value():(double)ui->sliderSpeeddeg->value();
     quint8 selected_sp_type = (mode_selection==true)?CMD_DATA_MOVE_SPEED_TYPE_V_SPEED:CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED;

     yrc1000micro_com.YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT,
         CMD_HEADER_MOVE_INSTANCE_LINE_INCREMENT, selected_sp_type, set_speed, &set_position);
}

void MainWindow::updateUICallback(bool signal_1, bool signal_2, bool signal_3)
{
    /*---------------------------------------------------------*/
                        /* Read Positions */
    //---Calibration Updates:
    if(read_pos)
    {
        robot_position = yrc1000micro_com.YRC1000microUpdateRobotPosition();
        read_pos = false;
    }
    //---UI Updates:
    robot_position = yrc1000micro_com.YRC1000microUpdateRobotPosition();
    ui->txtBoxX->setText(QString::number(robot_position[0]));
    ui->txtBoxY->setText(QString::number(robot_position[1]));
    ui->txtBoxZ->setText(QString::number(robot_position[2]));
    ui->txtBoxRoll->setText(QString::number(robot_position[3]));
    ui->txtBoxPitch->setText(QString::number(robot_position[4]));
    ui->txtBoxYaw->setText(QString::number(robot_position[5]));

    //---Config tab
    ui->txtBoxXConfig->setText(QString::number(robot_position[0]));
    ui->txtBoxYConfig->setText(QString::number(robot_position[1]));
    ui->txtBoxZConfig->setText(QString::number(robot_position[2]));
    ui->txtBoxRollConfig->setText(QString::number(robot_position[3]));
    ui->txtBoxPitchConfig->setText(QString::number(robot_position[4]));
    ui->txtBoxYawConfig->setText(QString::number(robot_position[5]));

    /*---------------------------------------------------------*/
                        /* Read Pulses */
    QVector<double> pulses = yrc1000micro_com.YRC1000microupdateRobotPulse();
    QVector<double> degs = convert_pulses_to_degs(pulses);
    ui->txtBoxJ1->setText(QString::number(degs[0]));
    ui->txtBoxJ2->setText(QString::number(degs[1]));
    ui->txtBoxJ3->setText(QString::number(degs[2]));
    ui->txtBoxJ4->setText(QString::number(degs[3]));
    ui->txtBoxJ5->setText(QString::number(degs[4]));
    ui->txtBoxJ6->setText(QString::number(degs[5]));

    /*---------------------------------------------------------*/
                        /* Read Status */
    quint8 running_status = yrc1000micro_com.YRC1000microUpdateStatus();

    if(running_status){
        ui->lblStatus->setText("Running");
    }
    else{
        ui->lblStatus->setText("Stop");
    }

    /*---------------------------------------------------------*/
    if(signal_2 && !signal_3 && ui->btnConnect->text() == "Disconnect"){
        QMessageBox::warning(this, "Job command error", "Failed to start job");
        yrc1000micro_com.job_start_status_error = false;
        signal_2 = false;
    }
    else if(signal_2 && signal_3 && ui->btnConnect->text() == "Disconnect"){
        QMessageBox::warning(this, "Job command error", "Job's not selected");
        yrc1000micro_com.job_start_status_error = false;
        signal_2 = false;
    }

}

void MainWindow::on_btnConnect_clicked()
{
   if(ui->btnConnect->text() == "Connect")
   {
        QHostAddress udp_address;
        quint16 udp_port;
        quint16 udp_file_port;

        QString ip_string = ui->txtBoxIPAddress->text();
        QStringList ip_list = ip_string.split(".");
        quint32 ip_int32 = (ip_list.at(0).toUInt() << 24) | (ip_list.at(1).toUInt() << 16)
                        | (ip_list.at(2).toUInt() << 8) | ip_list.at(3).toUInt();

        udp_address.setAddress(ip_int32);
        udp_port = ui->txtBoxPort->text().toUShort();
        udp_file_port = ui->txtBoxFilePort->text().toUShort();

        qDebug() << "Connect Successfully!";

        /*Set the parameter of connection for usage in yrc100micro_com:
        udp_address
        udp_port
        file_port   */
        yrc1000micro_com.YRC1000microSetConnection(udp_address,udp_port,udp_file_port);
        bool connection_satus = yrc1000micro_com.YRC1000microConnect();

        if(connection_satus == false)
        {
            QMessageBox::warning(this,"UDP Connection","Can not connect to UDP address!");
            return;
        }
        /*--- Enable all button ----*/
        ui->btnServoOn->setEnabled(1);
        ui->btnHome->setEnabled(1);
        ui->btnSelectJob->setEnabled(1);
        ui->btnStartJob->setEnabled(1);
        ui->txtBoxJobName->setEnabled(1);
        ui->btnGetPos->setEnabled(1);
        ui->btnTeach->setEnabled(1);
        ui->txtBoxposregister->setEnabled(1);
        ui->btnSetPos->setEnabled(1);
        /*-------------------------*/
        ui->btnServoOn->setPalette(QColorConstants::Svg::green);
        ui->btnConnect->setText("Disconnect");
        ui->btnConnect->setPalette(QColorConstants::Svg::red);

//    timer->start(2000);
//    pulsesTimer->start(2500);
//    dataRCVTimer->start(1200);
    }

    else if(ui->btnConnect->text() == "Disconnect")
    {
        ui->btnConnect->setText("Connect");
        ui->btnConnect->setPalette(QColorConstants::Svg::green);
        yrc1000micro_com.YRC1000microDisConnect();

        if(ui->btnServoOn->text() == "Servo Off")
        {
            yrc1000micro_com.YRC1000microOffServo();
            ui->btnServoOn->setText("Servo On");
            ui->btnServoOn->setPalette(QColorConstants::Svg::green);
        }

        /*--- Disable all button ----*/
        ui->btnServoOn->setEnabled(0);
        ui->btnHome->setEnabled(0);
        ui->btnSelectJob->setEnabled(0);
        ui->btnStartJob->setEnabled(0);
        ui->txtBoxJobName->setEnabled(0);
        ui->btnGetPos->setEnabled(0);
        ui->btnTeach->setEnabled(0);
        ui->txtBoxposregister->setEnabled(0);
        ui->btnSetPos->setEnabled(0);
        /*-------------------------*/
        QPalette pal = QPalette();
        pal.setColor(QPalette::Window, Qt::white);
        ui->btnServoOn->setPalette(pal);

    timer->stop();
    pulsesTimer->stop();
    dataRCVTimer->stop();
    }
}


void MainWindow::on_btnServoOn_clicked()
{
    if(ui->btnServoOn->text() == "Servo On" && ui->btnConnect->text() == "Disconnect"){
        yrc1000micro_com.YRC1000microOnServo();
        ui->btnServoOn->setText("Servo Off");
        ui->btnServoOn->setPalette(QColorConstants::Svg::red);
    }

    else if(ui->btnServoOn->text() == "Servo Off" && ui->btnConnect->text() == "Disconnect")
    {
        yrc1000micro_com.YRC1000microOffServo();
        ui->btnServoOn->setText("Servo On");
        ui->btnServoOn->setPalette(QColorConstants::Svg::green);
    }
}

void MainWindow::on_btnGetPos_clicked()
{
    yrc1000micro_com.YRC1000microReadPosition();
}


void MainWindow::on_btnSetPos_clicked()
{
    double set_speed;
    set_speed = ui->txtBoxSetSpeed->text().toDouble();
    QVector<double> set_position;

//  get position hệ trục xyz
    set_position.append(ui->txtBoxSetX->text().toDouble());
    set_position.append(ui->txtBoxSetY->text().toDouble());
    set_position.append(ui->txtBoxSetZ->text().toDouble());
    set_position.append(ui->txtBoxSetRoll->text().toDouble());
    set_position.append(ui->txtBoxSetPitch->text().toDouble());
    set_position.append(ui->txtBoxSetYaw->text().toDouble());

    /*MoveCartesian
     *@para-1: coordinate
     *@para-2: move_type
     *@para-3: speed_type
     *@para-4: speed, @para-5: position*/
    if(ui->rdBtnLinkAbs->isChecked())
        yrc1000micro_com.YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT,
            CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE, CMD_DATA_MOVE_SPEED_TYPE_LINK, set_speed, &set_position);

    else if(ui->rdBtnLineAbs->isChecked())
        yrc1000micro_com.YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT,
            CMD_HEADER_MOVE_INSTANCE_LINE_ABSOLUTE, CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED, set_speed, &set_position);

    else if(ui->rdBtnLineInc->isChecked())
        yrc1000micro_com.YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT,
            CMD_HEADER_MOVE_INSTANCE_LINE_INCREMENT, CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED, set_speed, &set_position);
}


void MainWindow::on_btnHome_clicked()
{
    double set_speed;
    set_speed = 60;
    QVector<double> home_position = QVector<double> (6);

    home_position[0] = 180.0;
    home_position[1] = 0.0;
    home_position[2] = 30.0;
    home_position[3] = 180.0;
    home_position[4] = 0.0;
    home_position[5] = 0.0;
    yrc1000micro_com.YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT,
        CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE, CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED, set_speed, &home_position);
}

/* 4 Functions for slider changed value*/
void MainWindow::on_sliderDismm_sliderMoved(int position)
{
    ui->lblDismm->setText(QString::number(position));
}
void MainWindow::on_sliderDisdeg_sliderMoved(int position)
{
    ui->lblDisdeg->setText(QString::number(position));
}
void MainWindow::on_sliderSpeedmm_sliderMoved(int position)
{
    ui->lblSpeedmm->setText(QString::number(position));
}
void MainWindow::on_sliderSpeeddeg_sliderMoved(int position)
{
    ui->lblSpeeddeg->setText(QString::number(position));
}
/*--------------------------------------------------------------------------------------------------*/

QVector<double> MainWindow::convert_pulses_to_degs(QVector<double> &pulses)
{
    QVector<double> degs(6);
    degs[0] = pulses.at(0)*30.0/34816.0;
    degs[1] = pulses.at(1)*90.0/102400.0;
    degs[2] = pulses.at(2)*90.0/51200.0;
    degs[3] = pulses.at(3)*30.0/10240.0;
    degs[4] = pulses.at(4)*30.0/10240.0;
    degs[5] = pulses.at(5)*30.0/10240.0;
    return degs;
}

//void MainWindow::on_btnGetJoints_clicked()
//{
//    yrc1000micro_com.YRC1000microReadPulse();
//}


//void MainWindow::on_btnCalcFK_clicked()
//{
//    QVector<double> input_theta(6);
//    QVector<double> output_result(6);
//    input_theta[0] = qDegreesToRadians(robot_pulse[0]);
//    input_theta[1] = qDegreesToRadians(ui->txtBoxJ2->text().toDouble());
//    input_theta[2] = qDegreesToRadians(ui->txtBoxJ3->text().toDouble());
//    input_theta[3] = qDegreesToRadians(ui->txtBoxJ4->text().toDouble());
//    input_theta[4] = qDegreesToRadians(ui->txtBoxJ5->text().toDouble());
//    input_theta[5] = qDegreesToRadians(ui->txtBoxJ6->text().toDouble());

//    yrc1000micro_com.forwardKinematic(&input_theta, &output_result);
//    qDebug() << output_result ;
//    ui->txtBoxX_FK->setText(QString::number(output_result[0],'f',4));
//    ui->txtBoxY_FK->setText(QString::number(output_result[1],'f',4));
//    ui->txtBoxZ_FK->setText(QString::number(output_result[2],'f',4));
//    ui->txtBoxRoll_FK->setText(QString::number(qRadiansToDegrees(output_result[3]),'f',4));
//    ui->txtBoxPitch_FK->setText(QString::number(qRadiansToDegrees(output_result[4]),'f',4));
//    ui->txtBoxYaw_FK->setText(QString::number(qRadiansToDegrees(output_result[5]),'f',4));
//}

void MainWindow::on_btnForwardKinematics_clicked()
{
        QVector<double> input_theta(6);
        QVector<double> output_result(6);
        input_theta[0] = qDegreesToRadians(ui->txtBoxJ1->text().toDouble());
        input_theta[1] = qDegreesToRadians(ui->txtBoxJ2->text().toDouble());
        input_theta[2] = qDegreesToRadians(ui->txtBoxJ3->text().toDouble());
        input_theta[3] = qDegreesToRadians(ui->txtBoxJ4->text().toDouble());
        input_theta[4] = qDegreesToRadians(ui->txtBoxJ5->text().toDouble());
        input_theta[5] = qDegreesToRadians(ui->txtBoxJ6->text().toDouble());

        yrc1000micro_com.forwardKinematic(&input_theta, &output_result);
        qDebug() << "X:" << output_result[0];
        qDebug() << "Y:" << output_result[1];
        qDebug() << "Z:" << output_result[2];
        qDebug() << "Rx:" << qRadiansToDegrees(output_result[3]);
        qDebug() << "Ry:" << qRadiansToDegrees(output_result[4]);
        qDebug() << "Rz:" << qRadiansToDegrees(output_result[5]);
}
/*--------------------------------------------------------------------------------------------*/
                                /*Serial Port Configuration*/
void MainWindow::on_btnSerOpen_clicked()
{
//    mSerial->setPortName("ttyUSB0");
    mSerial->setBaudRate(QSerialPort::Baud115200);
    mSerial->setDataBits(QSerialPort::Data8);
    mSerial->setParity(QSerialPort::NoParity);
    mSerial->setStopBits(QSerialPort::OneStop);
    QString serialLoc  =  ui->comboBoxPort->currentText();
    mSerial->setPortName(serialLoc);
//    mSerial->setBaudRate(static_cast<QSerialPort::BaudRate>(ui->comboBoxBaudrate->itemData(ui->comboBoxBaudrate->currentIndex()).toInt()));
//    mSerial->setDataBits(static_cast<QSerialPort::DataBits>(ui->comboBoxDatasize->itemData(ui->comboBoxDatasize->currentIndex()).toInt()));
//    mSerial->setParity(QSerialPort::NoParity);
//    mSerial->setStopBits(QSerialPort::OneStop);

    //mSerial->setFlowControl(QSerialPort::FlowControl);
    if(mSerial->open(QIODevice::ReadWrite))
    {
        msgBox.setText("Serial Port: Open");
        msgBox.exec();
        ui->btnSerOpen->setEnabled(false);
        ui->btnSerClose->setEnabled(true);
        ui->btnMotorRun->setEnabled(true);
        serFlag = true;
    }
    else
    {
        msgBox.critical(nullptr, tr("Error!"), mSerial->errorString());
        serFlag = false;
    }
}


void MainWindow::on_btnSerClose_clicked()
{
    if (mSerial->isOpen())
        {
            mSerial->close();
            msgBox.setText("Serial Port: Close");
            msgBox.exec();
            ui->btnSerOpen->setEnabled(true);
            ui->btnSerClose->setEnabled(false);
            ui->btnMotorRun->setEnabled(false);
            serFlag = false;
        }
}


void MainWindow::on_btnMotorRun_clicked()
{
    if(mSerial-> isOpen())
        {
            QString str = "RUN";
            str += ui ->txtBoxStep->text();
            str += ui ->txtBoxDelay->text();

            if ( ui->rdBtnDir->isChecked())
                str += '1';
            else
                str += '0';
            mSerial -> write(str.toLocal8Bit());
            qDebug() << "Send: " + str;
        }
     else
        {
            msgBox.critical(nullptr, tr("Transmit failed! "), mSerial->errorString());
        }
}
/*------------------------------------------------------------------------------------------------------------------------*/
                                             /*Job Command Configuration*/

void MainWindow::on_btnSelectJob_clicked()
{
    yrc1000micro_com.YRC1000microSelectJob(ui->txtBoxJobName->text());
}


void MainWindow::on_btnStartJob_clicked()
{
    yrc1000micro_com.YRC1000microStartJob();
}

/*------------------------------------------------------------------------------------------------------------------------*/

void MainWindow::on_btnTeach_clicked()
{
    int position_register = ui->txtBoxposregister->text().toInt();
    yrc1000micro_com.YRC1000microReadPosition();
    QVector<double> position = yrc1000micro_com.YRC1000microUpdateRobotPosition();
    yrc1000micro_com.YRC1000mircoTeachPosition(CMD_DATA_TEACH_POSITION_COORDINATE_TYPE_ROBOT, position_register, &position);
}
/*------------------------------------------------------------------------------------------------------------------------*/
                                            /*Computer Vision Configuration*/

void MainWindow::updateFrameandGui()
{
    cap >> frame;
    cvtColor(frame, frame ,COLOR_BGR2RGB); //Qt reads in RGB whereas CV in BGR
    QImage imdisplay((uchar*)frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888); //Converts the CV image into Qt standard format
    ui->lbldisplayFrameTest->setPixmap(QPixmap::fromImage(imdisplay));//display the image in label that is created earlier
}

void MainWindow::on_btnYOLO_clicked()
{
    camTimer->stop();
    Net net;
    QString qsModel = ui->txtBoxYOLOPath->text();
    string model = qsModel.toUtf8().constData();
    net = readNet(model);

    std::cout << "Detections: " << endl;
    vector<Mat> detections;
    cap >> frame;
    imshow("FRAME",frame);
    detections = tracker.pre_process(frame, net);
    Mat img = tracker.post_process(frame , detections, tracker.class_list);

    QString qsResult = ui->txtBoxResultIn->text();
    string result = qsResult.toUtf8().constData();
    imwrite("/home/phuc/Thesis/Result/" + result + ".png", img);

    //----Dislay detected frame
    cvtColor(img, img ,COLOR_BGR2RGB); //Qt reads in RGB whereas CV in BGR
    QImage imdisplay((uchar*)img.data, img.cols, img.rows, img.step, QImage::Format_RGB888); //Converts the CV image into Qt standard format
    ui->lbldisplayFrameTest->setPixmap(QPixmap::fromImage(imdisplay));//display the image in label that is created earlier
    cap.release();
}


void MainWindow::on_btnCamOnAuto_clicked()
{
    if(ui->btnCamOnAuto->text() == "Camera On")
    {
        QString qsIndex = ui->txtBoxCamIndex->text();
        int index = qsIndex.toInt();
        cap.open(index);
        if (!cap.isOpened()){
            QMessageBox::warning(this, "Camera Error!", "Failed to open camera");
        }

        else{
//            cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920.0); // valueX = your wanted width
//            cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080.0); // valueY = your wanted height
            camTimer = new QTimer(this);
            connect(camTimer, SIGNAL(timeout()), this, SLOT(updateFrameandGui()));
            camTimer->start(20);

            //---Which mode?
            if(ui->rdBtnYOLO->isChecked())
            {
                ui->btnYOLO->setEnabled(1);
                cap.set(cv::CAP_PROP_FRAME_WIDTH, 640); // valueX = your wanted width
                cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480); // valueY = your wanted height
                calibration.fullHDFlag = false;
            }
            else if(ui->rdBtnIP->isChecked())
            {
                ui->btnBG->setEnabled(1);
                ui->btnFG->setEnabled(1);
                ui->btnDetectIP->setEnabled(1);
                cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920); // valueX = your wanted width
                cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080); // valueY = your wanted height
                calibration.fullHDFlag = true;
            }
            ui->btnStartAuto->setEnabled(1);
            ui->btnStartAutoIP->setEnabled(1);
            ui->btnCamOnAuto->setText("Camera Off");
        }

    }
    else if(ui->btnCamOnAuto->text() == "Camera Off")
    {
        cap.release();
        camTimer->stop();
        ui->lbldisplayFrameTest->clear();
        ui->btnCamOnAuto->setText("Camera On");
        ui->btnYOLO->setDisabled(1);
        ui->btnDetectIP->setDisabled(1);
        ui->btnStartAuto->setDisabled(1);
        ui->btnStartAutoIP->setDisabled(1);
        ui->btnBG->setDisabled(1);
        ui->btnFG->setDisabled(1);
    }

}



void MainWindow::on_btnStartAuto_clicked()
{
//-------------------------------------------------------------------------------------------//
//    on_btnYOLO_clicked();
    cap >> frame;
    imshow("FRAME",frame);
    //---Converting with respect to central point
//    tracker.x_vec.push_back(0);
//    tracker.y_vec.push_back(0);

    double central_x = 327.09;
    double central_y = 240.459;
    double focalx = 606.281;
    double focaly = 606.418;

    QString tempQS;
    tempQS = ui->txtBoxXMarker->text();
    double marker_x = tempQS.toDouble();
    tempQS = ui->txtBoxYMarker->text();
    double marker_y = tempQS.toDouble();
    double z = 200; // mili
//    xPad_vec = tracker.x_vec;
//    yPad_vec = tracker.y_vec;
//    xPad_vec.at(0) = 405;
//    yPad_vec.at(0) = 333;
//    std::cout << "xPad: " << xPad_vec.at(0) << std::endl;
//    std::cout << "yPad: " << yPad_vec.at(0) << std::endl;
//    xPad_vec.at(0) = (xPad_vec.at(0) - central_x)*pixeltomm;
//    yPad_vec.at(0) = (yPad_vec.at(0) - central_y)*pixeltomm;
//    std::cout << "central_x: " << central_x << std::endl;
//    std::cout << "central_y: " << central_y << std::endl;
//    std::cout << "cxPad: " << xPad_vec.at(0) << std::endl;
//    std::cout << "cyPad: " << yPad_vec.at(0) << std::endl;

    std::cout << "xPad: " << marker_x << std::endl;
    std::cout << "yPad: " << marker_y << std::endl;

    marker_x = (z*(marker_x - central_x))/focalx;
    marker_y = (z*(marker_y - central_y))/focaly;

    std::cout << "cxPad: " << marker_x << std::endl;
    std::cout << "cyPad: " << marker_y << std::endl;

    //---Parameters-----------------------------------------------
    vpHomogeneousMatrix bTc, bTo, cTo, eTf, bTf;
    vpTranslationVector tV;
    vpRzyxVector rV;

    //---Get bTc---------------------------------------------------
    vpXmlParserHomogeneousMatrix pXML;

    tempQS = ui->txtBoxMatName->text();
    name_M = tempQS.toStdString();
    tempQS = ui->txtBoxMatFile->text();
    filename = tempQS.toStdString();

    if (pXML.parse(bTc, filename, name_M) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
        std::cout << "Cannot found the Homogeneous matrix named " << name_M << "." << std::endl;
    }
    else
        std::cout << "Homogeneous matrix " << name_M <<": " << std::endl << bTc << std::endl;

    tV = bTc.getTranslationVector();
    rV.buildFrom(bTc.getRotationMatrix());

    std::cout << "-----" << std::endl;
    std::cout << "bTc " << std::endl;
    std::cout << "Translation Vector: " << std::endl;
    std::cout << "X: " << tV[0]*1000 << std::endl;
    std::cout << "Y: " << tV[1]*1000 << std::endl;
    std::cout << "Z: " << tV[2]*1000 << std::endl;
    std::cout << "***" << std::endl;
    std::cout << "Rzyx vector: " << std::endl;
    std::cout << "Rz: " << vpMath::deg(rV[0]) << std::endl;
    std::cout << "Ry: " << vpMath::deg(rV[1]) << std::endl;
    std::cout << "Rx: " << vpMath::deg(rV[2]) << std::endl;

    //---Get cTo ------------------------------------------------------------------------
    double e,r,c;
    e = -179.461;
    r = -2.04589;
    c = -179.051;

    vpTranslationVector etc(marker_x/1000, marker_y/1000, z/1000);
//    vpTranslationVector etc(x/1000, y/1000, z/1000);
    vpRzyxVector erc(vpMath::rad(e),vpMath::rad(r) ,vpMath::rad(c));
    vpRotationMatrix rM;
    rM.buildFrom(erc);
    cTo.buildFrom(etc, rM);

    tV = cTo.getTranslationVector();
    std::cout << "cTo " << std::endl;
    std::cout << "Translation Vector: " << std::endl;
    std::cout << "X: " << tV[0]*1000 << std::endl;
    std::cout << "Y: " << tV[1]*1000 << std::endl;
    std::cout << "Z: " << tV[2]*1000 << std::endl;
    std::cout << "RzyxVector: " << std::endl;
    std::cout << "Rz: " << vpMath::deg(erc[0]) << std::endl;
    std::cout << "Ry: " << vpMath::deg(erc[1]) << std::endl;
    std::cout << "Rx: " << vpMath::deg(erc[2]) << std::endl;


    //---Get eTf ----
//    double x_f, y_f, z_f;
//    x_f = 65.0;
//    y_f = 4.0;
//    z_f = 85.754;
//    e = 0;
//    r = 0;
//    c = 0;
//    vpTranslationVector etc_f(x_f/1000, y_f/1000, z_f/1000);
//    vpTranslationVector etc(x/1000, y/1000, z/1000);
//    vpRzyxVector erc_f(vpMath::rad(e),vpMath::rad(r) ,vpMath::rad(c));
//    rM.buildFrom(erc_f);
//    eTf.buildFrom(etc_f, rM);

//    tV = eTf.getTranslationVector();
//    std::cout << "eTf " << std::endl;
//    std::cout << "Translation Vector: " << std::endl;
//    std::cout << "X: " << tV[0]*1000 << std::endl;
//    std::cout << "Y: " << tV[1]*1000 << std::endl;
//    std::cout << "Z: " << tV[2]*1000 << std::endl;
//    std::cout << "RzyxVector: " << std::endl;
//    std::cout << "Rz: " << vpMath::deg(erc[0]) << std::endl;
//    std::cout << "Ry: " << vpMath::deg(erc[1]) << std::endl;
//    std::cout << "Rx: " << vpMath::deg(erc[2]) << std::endl;
    //---Calculation----------------

    bTo = bTc * cTo;
    std::cout << "bTo: " << std::endl;
    std::cout << bTo << std::endl;

    //---Show bTo----------------
    tV = bTo.getTranslationVector();
    rV.buildFrom(bTo.getRotationMatrix());

    std::cout << "-----" << std::endl;
    std::cout << "bTo " << std::endl;
    std::cout << "Translation Vector: " << std::endl;
    std::cout << "X: " << tV[0]*1000 << std::endl;
    std::cout << "Y: " << tV[1]*1000 << std::endl;
    std::cout << "Z: " << tV[2]*1000 << std::endl;
    std::cout << "***" << std::endl;
    std::cout << "Rzyx vector: " << std::endl;
    std::cout << "Rz: " << vpMath::deg(rV[0]) << std::endl;
    std::cout << "Ry: " << vpMath::deg(rV[1]) << std::endl;
    std::cout << "Rx: " << vpMath::deg(rV[2]) << std::endl;
}


void MainWindow::on_btnStartAutoIP_clicked()
{
    //---Camera Parameters-------------------
    double central_x = intriPara_FullHD[2];
    double central_y = intriPara_FullHD[5];
    double focal_x = intriPara_FullHD[0];
    double focal_y = intriPara_FullHD[4];

    double X, Y;

    double xPad, yPad, cxPad, cyPad;
    int count;

    //---Parameters-----------------------------------------------
    vpHomogeneousMatrix bTc, bTo, cTo, bTc_X, bTc_Y;
    vpTranslationVector tV;
    vpRzyxVector rV;

    //-----------------------------Get bTc---------------------------------------------------

    vpXmlParserHomogeneousMatrix pXML;
    //=====================================================
    QString temp = ui->txtBoxMatName->text();
    name_M = temp.toStdString();
    temp = ui->txtBoxMatFile->text();
    filename = temp.toStdString();
   //=====================================================

    //    name_M = "bTc_5";
    if (pXML.parse(bTc_X, filename, name_M) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
        std::cout << "Cannot found the Homogeneous matrix named " << name_M << "." << std::endl;
    }
    else
        std::cout << "Homogeneous matrix " << name_M <<": " << std::endl << bTc_X << std::endl;

    //    name_M = "bTc_5";
    if (pXML.parse(bTc_Y, filename, name_M) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
        std::cout << "Cannot found the Homogeneous matrix named " << name_M << "." << std::endl;
    }
    else
        std::cout << "Homogeneous matrix " << name_M <<": " << std::endl << bTc_Y << std::endl;


    //--------------------------Get cTo ------------------------------------------------------------------------
    count = cxPad_vec.size();
    std::cout << "Size: " << cxPad_vec.size() << std::endl;
    //------------------------------Get The Object orientation according to the marker---
    //Set the tagsize:
    calibration.tagSize = 0.01; // 0.01 m
    on_btnGetPose_clicked();
    vpHomogeneousMatrix cTo_marker = calibration.getOnecMo();
    tV = cTo_marker.getTranslationVector();
    rV = rV.buildFrom(cTo_marker.getRotationMatrix());

    Z  = tV[2]*1000;
    Rz = rV[0];
    Ry = rV[1];
    Rx = rV[2];

    //    cout << "-----" << endl;
    //    cout << "cTo_marker " << endl;
    //    cout << "Translation Vector: " << endl;

    //    cout << "Z: " << Z << endl;
    //    cout << "***" << endl;
    //    cout << "Rzyx vector: " << endl;
    //    cout << "Rz: " << vpMath::deg(Rz) << endl;
    //    cout << "Ry: " << vpMath::deg(Ry) << endl;
    //    cout << "Rx: " << vpMath::deg(Rx) << endl;

    //---Calculation from 2D to 3D | the position of Pad to the principal point------------

    for(int i = 0; i < count; i++)
    {
        cxPad = cxPad_vec.at(i);
        cyPad = cyPad_vec.at(i);

        std::cout << "cxPad: " << cxPad << std::endl;
        std::cout << "cyPad: " << cyPad << std::endl;

        xPad = (Z*(cxPad - central_x))/focal_x;
        yPad = (Z*(cyPad - central_y))/focal_y;

        xPad_vec.push_back(xPad);
        yPad_vec.push_back(yPad);
    }

    //--------------------------------------------------------------------------
    for(int i = 0; i < count; i++)
    {
        vpTranslationVector xyz(xPad_vec.at(i)/1000, yPad_vec.at(i)/1000, Z/1000);
        vpRzyxVector r_zyx(Rz, Ry, Rx);
        vpRotationMatrix rM;

        rM.buildFrom(r_zyx);
        cTo.buildFrom(xyz, rM);
        //==================================================
        tV = cTo.getTranslationVector();
//        std::cout << "cTo " << std::endl;
//        std::cout << "Translation Vector: " << std::endl;
//        std::cout << "X: " << tV[0]*1000 << std::endl;
//        std::cout << "Y: " << tV[1]*1000 << std::endl;
//        std::cout << "Z: " << tV[2]*1000 << std::endl;
//        std::cout << "RzyxVector: " << std::endl;
//        std::cout << "Rz: " << vpMath::deg(r_zyx[0]) << std::endl;
//        std::cout << "Ry: " << vpMath::deg(r_zyx[1]) << std::endl;
//        std::cout << "Rx: " << vpMath::deg(r_zyx[2]) << std::endl;
        //==================================================

        //------------Calculation of X---------------
        std::cout << "========================" << std::endl;
        std::cout << "INDEX " << i + 1  << std::endl;
        bTo = bTc_X * cTo;

        //---Show bTo----------------
        tV = bTo.getTranslationVector();
        rV.buildFrom(bTo.getRotationMatrix());

        double x_offset = ui->txtBoxPrepX->text().toDouble(); //offset_val before: 0.854546 mm
        X = tV[0]*1000 + x_offset;

        std::cout << "X: " << X << std::endl;

        //------------Calculation of Y---------------

        bTo = bTc_Y * cTo;

        //---Show bTo----------------
        tV = bTo.getTranslationVector();
        rV.buildFrom(bTo.getRotationMatrix());

        double y_offset = ui->txtBoxPrepY->text().toDouble();

        Y = tV[1]*1000 + y_offset;
        std::cout << "Y: " << Y << std::endl;

        //-----------Calculation of Z----------------
        X_vec.push_back(X);
        Y_vec.push_back(Y);
    }

    //----Merge X&Y element into one vector coordinates. X-Y vec

    std::vector<vector<double>> XY_vec;

    for(int i = 0; i < count; i++)
    {
        vector<double> element;
        element.push_back(X_vec[i]);
        element.push_back(Y_vec[i]);
        XY_vec.push_back(element);
        element.clear();
    }

    qInfo() << "Sorting algorithm...";

//    for(int i = 0; i < count; i++)
//    {
//        std::cout << "------------------------" << std::endl;
//        std::cout << i << " X: " << XY_vec[i][0] << std::endl;
//        std::cout << i << " Y: " << XY_vec[i][1] << std::endl;
//    }

    for(int i = 0; i < count; i++)
    {
        for(int j = i + 1; j < count; j++)
        {
            if(XY_vec[i][0] > XY_vec[j][0])
            {
                double temp_X, temp_Y;
                temp_X = XY_vec[i][0];
                XY_vec[i][0] = XY_vec[j][0];
                XY_vec[j][0] = temp_X;
                //------------------------
                temp_Y = XY_vec[i][1];
                XY_vec[i][1] = XY_vec[j][1];
                XY_vec[j][1] = temp_Y;
            }
        }
    }

    for(int i = 0; i < count; i++)
    {
        std::cout << "----------Sorted------------" << std::endl;
        std::cout << i << " X: " << XY_vec[i][0] << std::endl;
        std::cout << i << " Y: " << XY_vec[i][1] << std::endl;
    }

    //------------------------------------------------------
    qInfo() << "Opening Serial Port...";

    if(!mSerial->isOpen())
    {
        on_btnSerOpen_clicked();
    }

    if(serFlag == true)
    {
        //---------------Call startSoldering Function--------------
        QVector<double> homePosition = QVector<double> (6);
        qInfo() << "%%% PREPARE TO SOLDER %%% ";
        std::cout << "PAD :" << count << std::endl;
        homePosition[0] = homeX;
        homePosition[1] = homeY;
        homePosition[2] = homeZ;
        homePosition[3] = ui->txtBoxPrepRoll->text().toDouble();
        homePosition[4] = ui->txtBoxPrepPitch->text().toDouble();
        homePosition[5] = ui->txtBoxPrepYaw->text().toDouble();
        //----Get the decided height for soldering
        double Z = ui->txtBoxPrepZ->text().toDouble();
        //--------Run the automated function in a different thread.
        QFuture<void> solderThread = QtConcurrent::run(&this->yrc1000micro_com, &YRC1000micro_com::startSoldering, count, XY_vec , Z, homePosition);
    }
    //--------------Clear Data--------------------------------
    cxPad_vec.clear();
    cyPad_vec.clear();
    xPad_vec.clear();
    yPad_vec.clear();
    X_vec.clear();
    Y_vec.clear();
}



void MainWindow::on_btnDetectIP_clicked()
{
    //----Clear previous data------
    cxPad_vec.clear();
    cyPad_vec.clear();
    //-----------------------------
    QString qs;
    string s;
    Mat bg_img, fg_img;
    //-----Read background img---------------
    qs = ui->txtBoxBG->text();
    s  = qs.toUtf8().constData();

    bg_img = imread(s);
    //-----Read front img---------------
    qs = ui->txtBoxFG->text();
    s  = qs.toUtf8().constData();
    fg_img = imread(s);

    //---Detect using background substraction
    if(ui->comboBoxMethod->currentText() == "bgSubstraction"){
        qInfo() << "Background substraction" ;
        tracker.bgSubtraction(bg_img, fg_img);
        cxPad_vec = tracker.getcxPadVec();
        cyPad_vec = tracker.getcyPadVec();
    }
    else if(ui->comboBoxMethod->currentText() == "blobDetection")
    {
        qInfo() << "Blob Detection" ;
        tracker.blobDetection(bg_img);
        cxPad_vec = tracker.getcxPadVec();
        cyPad_vec = tracker.getcyPadVec();
    }
    //------Clear tracker cxVec, cyVec for later data
    tracker.clearPadvec();
}

void MainWindow::on_btnFG_clicked()
{
    cv::Mat fg_img;
    cap >> fg_img;

    if(fg_img.empty())
    {
      std::cout << "Something is wrong with the webcam, could not get frame." << std::endl;
    }

    QString qsFG = ui->txtBoxFG->text();
    string sFG = qsFG.toUtf8().constData();
    imwrite(sFG, fg_img);
    ui->btnDetectIP->setEnabled(1);
}


void MainWindow::on_btnBG_clicked()
{
    cv::Mat bg_img;
    cap >> bg_img;
    if(bg_img.empty())
    {
      std::cout << "Something is wrong with the webcam, could not get frame." << std::endl;
    }

    QString qsBG = ui->txtBoxBG->text();
    string sBG = qsBG.toUtf8().constData();
    cv::imwrite(sBG, bg_img);
//    cv::imwrite("/home/phuc/BG/fg_img.jpg", bg_img);
}


/*------------------------------------------------------------------------------------------------------------------------*/
                                            /*Configuration Tab*/
void MainWindow::on_btnBrowseYOLO_clicked()
{
    QString qsPath = QFileDialog::getOpenFileName(this, "Open YOLO Model", "Model (*.onnx)");
    ui->txtBoxYOLOPath->setText(qsPath);
}



/*------------------------------------------------------------------------------------------------------------------------*/
                                            /*Calibration*/
void MainWindow::updateFrameConfig()
{
    cap >> frame;
//    std::cout << frame.cols << std::endl;
//    std::cout << frame.rows << std::endl;
//    if(count <= refCount - 2)
//    {
//        imshow("1080",frame);
//        waitKey(2);
//    }
    cvtColor(frame, frame ,COLOR_BGR2RGB); //Qt reads in RGB whereas CV in BGR
    QImage imdisplay((uchar*)frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888); //Converts the CV image into Qt standard format
    ui->lbldisplayFrameConfig->setPixmap(QPixmap::fromImage(imdisplay));//display the image in la bel*/ that is created earlier
}

void MainWindow::on_btnCamOnConfig_clicked()
{
    if(ui->btnCamOnConfig->text() == "Camera On")
    {
        QString qsIndex = ui->txtBoxCamIndexConfig->text();
        int index = qsIndex.toInt();
        cap.open(index);
        if (!cap.isOpened())
        {
            QMessageBox::warning(this, "Camera Error!", "Failed to open camera");
        }

        else
        {
            //---Which mode?
            if(ui->rdBtnYOLO->isChecked())
            {
                cap.set(cv::CAP_PROP_FRAME_WIDTH, 640); // valueX = your wanted width
                cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480); // valueY = your wanted height
                calibration.fullHDFlag = false;
            }
            else if(ui->rdBtnIP->isChecked())
            {
                cap.set(cv::CAP_PROP_FRAME_WIDTH, 1920); // valueX = your wanted width
                cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1080); // valueY = your wanted height
                calibration.fullHDFlag = true;
            }

            //--------------
            QString qsCount = ui->txtBoxCalibCount->text();
            refCount = qsCount.toInt();

            camConfigTimer = new QTimer(this);
            connect(camConfigTimer, SIGNAL(timeout()), this, SLOT(updateFrameConfig()));
            camConfigTimer->start(5);
            ui->btnCamOnConfig->setText("Camera Off");
        }

        camConfigTimer->start(5);
        ui->btnCamOnConfig->setText("Camera Off");
    }
    else if(ui->btnCamOnConfig->text() == "Camera Off")
    {
        cap.release();
        camConfigTimer->stop();
        ui->lbldisplayFrameConfig->clear();
        ui->btnCamOnConfig->setText("Camera On");
    }

}

void MainWindow::calibTimerCallback()
{

}

void MainWindow::on_btnGetPose_clicked()
{
    qDebug() << "Start Getting Pose" ;
//    QString qsIndex = ui->txtBoxCamIndexConfig->text();
//    int index = qsIndex.toInt();
//    cap.open(index);
    calibration.poseFlag = false;

    while(!calibration.poseFlag)
    {
        //---Webcam
        cap >> frame;
        calibration.getPose(frame);
        //---Intel realsense
//        calibration.getPose_RealSense();
        delay->delay(300);
    }
}

void MainWindow::on_btnStartCalib_clicked()
{
    ui->btnStartCalib->setDisabled(1);
    //---Variables---

    //---Repeat xx times
    cout << "++++++++" << endl;
    cout << "i = " << ++count << endl;

    //---Get Pose
    on_btnGetPose_clicked();
    vpHomogeneousMatrix cTo = calibration.getOnecMo();

    //---Write to text file:
    //-Pose
    QString tempFile;
    tempFile = ui->txtBoxposeFile->text();
    QFile file("/home/phuc/Thesis/POSE/" + tempFile + timeS + ".txt");

    if(!file.open(QIODevice::ReadWrite | QIODevice::Append))
    {
        qCritical() << "Couldn't write the file";
        qCritical() << file.errorString();
    }
    else{
        QTextStream stream(&file);
        {
            vpRzyxVector rV;
            vpTranslationVector tV;
            tV = cTo.getTranslationVector();
            rV = rV.buildFrom(cTo.getRotationMatrix());
//            stream << count << "\n";
            stream <<  tV[0] << "," << tV[1] << "," << tV[2] << "," << rV[0] << "," << rV[1] << "," << rV[2] << "," << '\n';
            file.close();
        }
    }

    //---------------------------
    //---Get to April Tag position
    yrc1000micro_com.YRC1000microReadPosition();
    read_pos = true;

    while(read_pos)
    {
        delay->delay(500);
    }

    tempFile = ui->txtBoxRobotPosFile->text();
    QFile file_pos("/home/phuc/Thesis/POSE/" + tempFile + timeS + ".txt");

    if(!file_pos.open(QIODevice::ReadWrite | QIODevice::Append))
    {
        qCritical() << "Couldn't write the file";
        qCritical() << file_pos.errorString();
    }
    else{
        QTextStream stream(&file_pos);
        {
//            stream << count << "\n";
            stream <<  robot_position[0]/1000 << "," << robot_position[1]/1000 << "," << robot_position[2]/1000 << "," << vpMath::rad(robot_position[5]) << "," << vpMath::rad(robot_position[4]) << "," << vpMath::rad(robot_position[3]) << "," << '\n';
            file_pos.close();
        }
    }

    //---Call hand-eye-calibration

    if ( count == refCount)
    {
        calibration.calibrationFlag = true;
        count = 0;
        //---Saving for later assessment
        cTo_check = cTo;

        vpTranslationVector etc(robot_position[0]/1000, robot_position[1]/1000, robot_position[2]/1000);
        vpRzyxVector erc(vpMath::rad(robot_position[5]),vpMath::rad(robot_position[4]) ,vpMath::rad(robot_position[3]));
        vpRotationMatrix ERC;
        ERC.buildFrom(erc);

        vpHomogeneousMatrix bTe;
        bTe.buildFrom(etc, ERC);
        bTe.inverse(eTb_check);

        std::cout << "eTb_check: " << eTb_check << std::endl;

        ui->btnSaveFile->setEnabled(1);
    }
    calibration.handEyeCalibration(robot_position, cTo);

    robot_position.clear();
    ui->btnBG->setEnabled(1);
    ui->btnStartCalib->setEnabled(1);
}

void MainWindow::on_btnArucoDetection_clicked()
{
    if (ui->btnArucoDetection->text() == "ArUco")
    {
        qDebug() << "Start Detecting ArUco" ;
        QString qsIndex = ui->txtBoxCamIndexConfig->text();
        int camIndex = qsIndex.toInt(); // For OpenCV
        calibration.arucoDetection(camIndex);
        cv::Mat frame = calibration.getUIImage();
        cvtColor(frame, frame ,COLOR_BGR2RGB); //Qt reads in RGB whereas CV in BGR
        QImage imdisplay((uchar*)frame.data, frame.cols, frame.rows, frame.step, QImage::Format_RGB888); //Converts the CV image into Qt standard format
        ui->lbldisplayFrameConfig->setPixmap(QPixmap::fromImage(imdisplay));//display the image in label that is created earlier
        //---Change text button
        ui->btnArucoDetection->setText("Delete");
    }
    else if (ui->btnArucoDetection->text() == "Delete")
    {
        ui->lbldisplayFrameConfig->clear();
        //---Change text button
        ui->btnArucoDetection->setText("ArUco");
    }
}

void MainWindow::on_btnIntriCalib_clicked()
{
#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    std::cout << "have cxx 11 " << std::endl;
#endif
    calibration.intrinCalibration();

}

void MainWindow::on_btnTEST_clicked()
{
    //---Testing on ArilTag---------
    vpHomogeneousMatrix bTc, bTo, cTo, bTe;
    vpTranslationVector tV;
    vpRzyxVector rV;
    //---Get bTc---------
    vpXmlParserHomogeneousMatrix pXML;

    //---Define the name of the matrix to load
    QString temp = ui->txtBoxMatName->text();

    name_M = temp.toStdString();
    temp = ui->txtBoxMatFile->text();
    qInfo() << "Openning File:" << temp ;
    filename = temp.toStdString();

    if (pXML.parse(bTc, filename, name_M) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
        std::cout << "Cannot found the Homogeneous matrix named " << name_M << "." << std::endl;
    }
    else
        std::cout << "Homogeneous matrix " << name_M <<": " << std::endl << bTc << std::endl;

    tV = bTc.getTranslationVector();
    rV.buildFrom(bTc.getRotationMatrix());

    std::cout << "-----" << std::endl;
    std::cout << "bTc " << std::endl;
    std::cout << "Translation Vector: " << std::endl;
    std::cout << "X: " << tV[0]*1000 << std::endl;
    std::cout << "Y: " << tV[1]*1000 << std::endl;
    std::cout << "Z: " << tV[2]*1000 << std::endl;
    std::cout << "***" << std::endl;
    std::cout << "Rzyx vector: " << std::endl;
    std::cout << "Rz: " << vpMath::deg(rV[0]) << std::endl;
    std::cout << "Ry: " << vpMath::deg(rV[1]) << std::endl;
    std::cout << "Rx: " << vpMath::deg(rV[2]) << std::endl;

    //---Get Pose------------------

    on_btnGetPose_clicked();
    cTo = calibration.getOnecMo();
//    std::cout << "cTo " << cTo << std::endl;
    //---Calculation----------------
    vpHomogeneousMatrix eTo, oTe;
    eTo = eTb_check * bTc * cTo_check;
    std::cout << "eTo has be constant: " << std::endl;
    std::cout << eTo << std::endl;
    eTo.inverse(oTe);
    bTe = bTc * cTo * oTe;
    //------------------------------
    std::cout << "For checking extrinsics Matrix---- " << std::endl;
    std::cout << "bTe: " << std::endl;
    std::cout << bTe << std::endl;

    //---Show bTe for checking----------------
    tV = bTe.getTranslationVector();
    rV.buildFrom(bTe.getRotationMatrix());

    std::cout << "-----" << std::endl;
    std::cout << "bTe " << std::endl;
    std::cout << "Translation Vector: " << std::endl;
    std::cout << "X: " << tV[0]*1000 << std::endl;
    std::cout << "Y: " << tV[1]*1000 << std::endl;
    std::cout << "Z: " << tV[2]*1000 << std::endl;
    std::cout << "***" << std::endl;
    std::cout << "Rzyx vector: " << std::endl;
    std::cout << "Rz: " << vpMath::deg(rV[0]) << std::endl;
    std::cout << "Ry: " << vpMath::deg(rV[1]) << std::endl;
    std::cout << "Rx: " << vpMath::deg(rV[2]) << std::endl;
}


void MainWindow::on_btnSaveFile_clicked()
{
    //---Define the name of the matrix
    QString name_MQS = ui->txtBoxMatName->text();
    name_M = name_MQS.toUtf8().constData();

    // Define name of the file xml to fill
//            filename = "extrinsics_homogeneous_matrixes.xml";
    QString filenameQS = ui->txtBoxMatFile->text();
    qInfo() << "Saving to file: " << filenameQS;
    filename= filenameQS.toUtf8().constData();

    if(!ui->rdBtnHandeye->isChecked())
    {
            bTc = calibration.getbTc();
    }

    //======================CUSTOM SAVING====================

//    double x, y, z;
//    x = -0.0120;
//    y = -0.2763;
//    z = 0.2188;
//    double rx , ry, rz;
//    rx = 2.9398;
//    ry = 0.0005;
//    rz = 1.5988;

//    vpTranslationVector etc(x, y, z);
//    vpRzyxVector erc(rz, ry, rx);
//    vpRotationMatrix ERC;
//    ERC.buildFrom(erc);
//    bTc.buildFrom(etc, ERC);

    //=========================================================
    if (pXML.save(bTc, filename, name_M) != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
      std::cout << "Cannot save the Homogeneous matrix" << std::endl;
    }
    else
    {
        ui->btnSaveFile->setDisabled(1);
    }
}



bool MainWindow::writeFile(QString fileName)
{
    QFile file("/home/phuc/Thesis/POSE/" + fileName + ".txt");
    if(!file.open(QIODevice::ReadWrite))
    {
        qCritical() << "Couldn't write the file";
        qCritical() << file.errorString();
    }
    qInfo() << "Writing file..." ;

    QTextStream stream(&file);
    {
        stream << "Hello world\r\n" ;
    }
    file.close();
    return 0;
}



void MainWindow::on_btnCalculate_clicked()
{
    qDebug() << "Pending for handeye calculation" ;
    double x, y ,z;
    double rz, ry, rx;
    QString temp;

    QString tempFile;
    tempFile = ui->txtBoxposeFile->text();
    QFile inputFile("/home/phuc/Thesis/POSE/" + tempFile + ".txt");

    //---Camera to Object text
    if(!inputFile.open(QIODevice::ReadOnly))
    {
        qCritical() << "Couldn't read the file";
        qCritical() << inputFile.errorString();
    }
    else
    {
        QTextStream in(&inputFile);
        QString line;
        while (!in.atEnd())
        {
            line = in.readLine();
            QStringList fields = line.split(",");
            //---Converting---------------------
            temp = fields.at(0);
            x = temp.toDouble();
            temp = fields.at(1);
            y = temp.toDouble();
            temp = fields.at(2);
            z = temp.toDouble();

            temp = fields.at(3);
            rz = temp.toDouble();
            temp = fields.at(4);
            ry = temp.toDouble();
            temp = fields.at(5);
            rx = temp.toDouble();

            vpTranslationVector etc(x, y, z);
            vpRzyxVector erc(rz, ry, rx);
            vpRotationMatrix ERC;
            ERC.buildFrom(erc);

            vpHomogeneousMatrix cTo;
            cTo.buildFrom(etc, ERC);
            cTo_vec.push_back(cTo);
            //---Converting---------------------
        }
        inputFile.close();
    }


    //---Robot Position text
    tempFile = ui->txtBoxRobotPosFile->text();

    QFile inputFile_pos("/home/phuc/Thesis/POSE/" + tempFile + ".txt");

    if(!inputFile_pos.open(QIODevice::ReadOnly))
    {
        qCritical() << "Couldn't read the pose file";
        qCritical() << inputFile_pos.errorString();
    }
    else
    {
        QTextStream in(&inputFile_pos);
        QString line;
        while (!in.atEnd())
        {
            //---Converting---------------------
            line = in.readLine();
            QStringList fields = line.split(",");
            //---Converting---------------------

            temp = fields.at(0);
            x = temp.toDouble();
            temp = fields.at(1);
            y = temp.toDouble();
            temp = fields.at(2);
            z = temp.toDouble();
            temp = fields.at(3);
            rz = temp.toDouble();
            temp = fields.at(4);
            ry = temp.toDouble();
            temp = fields.at(5);
            rx = temp.toDouble();

            vpTranslationVector etc(x, y, z);
            vpRzyxVector erc(rz, ry, rx);
            vpRotationMatrix ERC;
            ERC.buildFrom(erc);

            vpHomogeneousMatrix bTe, eTb;
            bTe.buildFrom(etc, ERC);
            bTe.inverse(eTb);
            eTb_vec.push_back(eTb);

            //---Converting---------------------
        }
        inputFile.close();
    }

    //-----Begin to calculate handeye calibration

    if (ui->rdBtnHandeye->isChecked())
    {
        //-------OPENCV HANDEYE----------------------------------
        //-------------------------------------------------------
        std::cout << "Calculating %HandEyeCalibration% ..." << endl;
        int ret = vpHandEyeCalibration::calibrate(cTo_vec, eTb_vec, bTc);
        std::cout << "ret " << ret << std::endl;
        std::cout << "bTc: " << std::endl;
        std::cout << bTc << std::endl;
        vpTranslationVector tV;
        vpRzyxVector rV;
        //---Showing bTc
        tV = bTc.getTranslationVector();
        rV = rV.buildFrom(bTc.getRotationMatrix());

        std::cout << "-----" << endl;
        std::cout << "bTc " << endl;
        std::cout << "Translation Vector: " << endl;
        std::cout << "X: " << tV[0]*1000 << endl;
        std::cout << "Y: " << tV[1]*1000 << endl;
        std::cout << "Z: " << tV[2]*1000 << endl;
        std::cout << "***" << endl;
        std::cout << "Rzyx vector: " << endl;
        std::cout << "Rz: " << vpMath::deg(rV[0]) << endl;
        std::cout << "Ry: " << vpMath::deg(rV[1]) << endl;
        std::cout << "Rx: " << vpMath::deg(rV[2]) << endl;

        //-----Saving file
        ui->btnSaveFile->setEnabled(1);

        delay->delay(500);
        on_btnSaveFile_clicked();

        ui->rdBtnHandeye->setChecked(0);
        bTc.eye();
        cTo_vec.clear();
        eTb_vec.clear();
    }
}

//==========================================================================================================

