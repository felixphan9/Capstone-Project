#ifndef YRC1000MICRO_COM_H
#define YRC1000MICRO_COM_H
#include <QObject>
#include <QFile>
#include <QTimer>
#include "udp.h"
#include <QVector>
#include <QMatrix4x4>
#include <QList>
#include <QtMath>
#include <unistd.h>
#include "yrc1000micro_command.h"
#include <QMessageBox>
#include <QThread>
#include <iostream>
#include "qserialport.h"
class YRC1000micro_com : public QObject
{
    Q_OBJECT

signals:
    void dataUIReceiveSignal(bool, bool, bool);
    void selectJobSignal(QString);
    void motorRun();

public slots:
//    Receive data function.
    void YRC1000microDataCallback();

public:
    explicit YRC1000micro_com(QObject *parent = nullptr); //constructor
    ~YRC1000micro_com(); //destructor
    //---Function Calibration Data Reading

//    Connection and Set; connection infor
    bool YRC1000microConnect();
    void YRC1000microSetConnection(QHostAddress address,quint16 port,quint16 file_port);
    void YRC1000microDisConnect();

//    Function for reading robot status, position, pulse
    void YRC1000microReadStatus();
    void YRC1000microReadPosition();
    void YRC1000microReadPulse();
    quint8 YRC1000microUpdateStatus();

//  ---Function for data processing.
    void YRC1000microReadPositionResponse(QByteArray data);
    void YRC1000microReadStatusResponse(QByteArray data);
    void YRC1000microReadPulseResponse(QByteArray data);

//  ---Command for robot
    void YRC1000microOnServo();
    void YRC1000microOffServo();
    void YRC1000microSetPos();
    void YRC1000microMoveCartesian(quint8 coordinate,quint8 move_type,
          quint8 speed_type, double speed, QVector<double>* position);

//  ---Get the robot_position para
    QVector<double> YRC1000microUpdateRobotPosition();
    QVector<double> YRC1000microupdateRobotPulse();

//  ---Function for calculate forward - inverse kinematics
    void forwardKinematic(QVector<double>*theta, QVector<double>*output_cartesian);
    void inverseKinematic(QVector<double>*theta, QVector<double>*output_cartesian);

//  ---Basic Control for Job
    void YRC1000microSelectJob(QString);
    void YRC1000microSelectJobResponse(QByteArray);

    void YRC1000microStartJob();
//  ---Teach robot position
    void YRC1000mircoTeachPosition(quint8, int, QVector<double> *);
    void YRC1000mircoTeachPositionResponse(QByteArray);
//  ---Homogeneous Matrix
    QMatrix4x4 getbTe();
//  ---Soldering Task.
    void startSoldering(uint count, std::vector<std::vector<double>> XY_vec, double Z , QVector<double> homePosition);

public:
    //  ---Boolean variables
        bool read_pos;
private:
    UDP udp_server;
    YRC1000micro_command yrc1000micro_command;


//  ---Request_ID_Index -> to keep track of the ID_COMMAND
    quint64 request_id_index;
//  Request_code array store all the request command with the index for each command
//  is request_id_index, request_code[request_id_index]
    char32_t request_code[25600] = {};

//  ---UDP Socket infor para
    QHostAddress udp_address;
    quint16 udp_port;
    quint16 udp_file_port;

//  ---Robot position para
//    QList<double> robot_position = QList<double> (6);
//    QList<double> robot_pulse = QList<double> (6);
    QVector<double> robot_position = QVector<double> (6);
    QVector<double> robot_pulse = QVector<double> (6);
    quint32 robot_status;
    quint8 running_status;

    int read_position_type;
//  ---Data buffer cmd = header_to_send + data_to_send
    QByteArray cmd;
    QMatrix4x4 bTe;
    QString job;

    //---Delay -------
    unsigned int microsecond = 1000000;
public:
    bool job_select_status_error;
    bool job_start_status_error;
    bool servo_status;
    bool teach_signal_error;

};

#endif // YRC1000MICRO_COM_H
