#include "yrc1000micro_command.h"
#include <QVector>
#include <QDebug>

YRC1000micro_command::YRC1000micro_command(QObject *parent) : QObject(parent)
{
    /*Init header and data for send command*/
    //Init header
    header_to_send.append('\x59'); //Y
    header_to_send.append('\x45'); //E
    header_to_send.append('\x52'); //R
    header_to_send.append('\x43'); //C
    header_to_send.append('\x20'); //Header size
    header_to_send.append('\x00');
    header_to_send.append('\x04'); //Data part size
    header_to_send.append('\x00');
    header_to_send.append('\x03'); //reserve 1
    header_to_send.append('\x01'); //Processing division:  1 robot control, 2: file control
    header_to_send.append('\x00'); //ACK: 0 request, 1 other than request.

    header_to_send.append('\x00'); //request ID
    header_to_send.append('\x00'); //block No[0]
    header_to_send.append('\x00'); //block No[1]
    header_to_send.append('\x00'); //block No[2]
    header_to_send.append('\x00'); //block No[3]
    header_to_send.append('\x39');
    header_to_send.append('\x39');
    header_to_send.append('\x39');
    header_to_send.append('\x39');
    header_to_send.append('\x39');
    header_to_send.append('\x39');
    header_to_send.append('\x39');
    header_to_send.append('\x39');
    header_to_send.append('\x83'); //command No[0] // [24]
    header_to_send.append('\x00'); //command No[1]
    header_to_send.append('\x02'); //instance  [0]
    header_to_send.append('\x00'); //instance  [1]
    header_to_send.append('\x01'); //attribute
    header_to_send.append('\x10'); //service
    header_to_send.append('\x00');
    header_to_send.append('\x00');

    //Init data------------------
    data_to_send.append('\x00');
    data_to_send.append('\x00');
    data_to_send.append('\x00');
    data_to_send.append('\x00');

    //  Init move data
    for(int i=0;i<104;i++)
    {
        move_cartesian_data.append('\x00');
    }

    // Init job data part
    for(int i = 0; i < 36; i++){
        job_to_send.append('\x00');
    }

    // Init teach position data part
    for(int i = 0; i < 52; i++){
        teach_position_data_part.append('\x00');
    }

    //DATA_MOVE_ROBOT_NUMBER move_cartesian_data[0] = 0x01
    //DATA_MOVE_USER_COORDINATE move_cartesian_data[64] = 0x01
    move_cartesian_data[0] = '\x01';
    move_cartesian_data[64] = '\x01';
}

YRC1000micro_command::~YRC1000micro_command(){
}


QByteArray YRC1000micro_command::ReadStatus()
{
//    Want to answer as data_1 or data_2
//    quint8 ATTRIBUTE_1 = '\x01';
//    quint8 ATTRIBUTE_2 = '\x02';

    QByteArray cmd = header_to_send;
    cmd[CMD_DATA_SIZE] = '\x00';
    cmd[CMD_ID_NO]     = CMD_ID_STATUS_READING;
    cmd[CMD_INSTANCE]  = '\x01';
    cmd[CMD_ATTRIBUTE] = '\x01';
    cmd[CMD_SERVICE]   = '\x01';
    return cmd;
}

QByteArray YRC1000micro_command::OffServo()
{
    QByteArray cmd = header_to_send + data_to_send;

    cmd[CMD_INSTANCE] = 0x02;
    cmd[CMD_ATTRIBUTE] = 0x01;
    cmd[CMD_SERVICE] = 0x10;
    cmd[CMD_ID_NO] = CMD_ID_SERVO_ON;

    //  Change the Data part -> 0x02 for OFF command
    cmd[DATA_BYTE0] = CMD_DATA_SERVO_OFF;
    return cmd;
}


QByteArray YRC1000micro_command::OnServo()
{
    QByteArray cmd = header_to_send + data_to_send;

    cmd[CMD_INSTANCE] = 0x02;
    cmd[CMD_ATTRIBUTE] = 0x01;
    cmd[CMD_SERVICE] = 0x10;

//  Change the Data part -> 0x01 for ON command
    cmd[CMD_ID_NO] = CMD_ID_SERVO_ON;
    cmd[DATA_BYTE0] = '\x01';
    return cmd;
}


QByteArray YRC1000micro_command::ReadPosition()
{
    QByteArray cmd = header_to_send;

    /*Change the command for readpos()*/
    cmd[CMD_DATA_SIZE] = '\x00'; //Don't need data size in reading/receiving command.

    cmd[CMD_ID_NO] = CMD_ID_READ_POS;
    cmd[CMD_INSTANCE] = '\x65';   //0x65 = 101 reading cartesian coordinate
    cmd[CMD_ATTRIBUTE] = '\x00';
    cmd[CMD_SERVICE] = '\x01';  //0x01 Read out all data of all the element number

    return cmd;
}

QByteArray YRC1000micro_command::ReadPulse()
{
    QByteArray cmd = header_to_send;

    /*Change the command for readpos()*/
    cmd[CMD_DATA_SIZE] = '\x00'; //Don't need data size in reading/receiving command.

    cmd[CMD_ID_NO] = CMD_ID_READ_POS;
    cmd[CMD_INSTANCE] = '\x01';   //0x01 reading robot joint pulses
    cmd[CMD_ATTRIBUTE] = '\x00'; // get all attribute
    cmd[CMD_SERVICE] = '\x01';  //0x01 Read out all data of all the element number
    return cmd;
}

QByteArray YRC1000micro_command::setRobotPositionCartesian(quint8 coordinate, quint8 move_type,
                                                   quint8 speed_type, double speed, QVector<double>* position)
{
    QByteArray cmd = header_to_send + move_cartesian_data;

    cmd[CMD_DATA_SIZE] = CMD_HEADER_MOVE_CARTESIAN_DATA_SIZE;
    cmd[CMD_ID_NO] = CMD_ID_MOVE_ROBOT_CARTESIAN;
    cmd[CMD_INSTANCE] = move_type;
    cmd[CMD_ATTRIBUTE] = CMD_HEADER_MOVE_ATTRIBUTE;
    cmd[CMD_SERVICE] = CMD_HEADER_MOVE_SERVICE_ALL;

    if(speed < 0) speed = 0;

    quint32 speed_u= (quint32)(speed*10);
    quint32 x_u = (quint32)(position->at(0)*1000);
    quint32 y_u = (quint32)(position->at(1)*1000);
    quint32 z_u = (quint32)(position->at(2)*1000);
    quint32 roll_u = (quint32)(position->at(3)*10000);
    quint32 pitch_u = (quint32)(position->at(4)*10000);
    quint32 yaw_u = (quint32)(position->at(5)*10000);

//    qDebug() << "X_U" << x_u;
//    qDebug() << "Y_U" << y_u;
//    qDebug() << "Z_U" << z_u;
//    qDebug() << "Roll_U" << roll_u;
//    qDebug() << "Pitch_U" << pitch_u;
//    qDebug() << "Yaw_U" << yaw_u;
//    qDebug() << "Speed " << speed_u;

    /* LINK_ABSOLUTE speed_type = 0x00
     * Coordinate 0x11 = 17: robot_coordinate*/

    cmd[DATA_MOVE_SPEED_TYPE + HEADER_SIZE] = speed_type;
    cmd[DATA_MOVE_COORDINATE + HEADER_SIZE]= coordinate;

    cmd[DATA_MOVE_SPEED + HEADER_SIZE+3]= (quint8)(speed_u>>24);
    cmd[DATA_MOVE_SPEED + HEADER_SIZE+2]= (quint8)(speed_u>>16);
    cmd[DATA_MOVE_SPEED + HEADER_SIZE+1]= (quint8)(speed_u>>8);
    cmd[DATA_MOVE_SPEED + HEADER_SIZE]= (quint8)(speed_u);

    cmd[DATA_MOVE_X_CARTESIAN + HEADER_SIZE+3]= (quint8)(x_u>>24);
    cmd[DATA_MOVE_X_CARTESIAN + HEADER_SIZE+2]= (quint8)(x_u>>16);
    cmd[DATA_MOVE_X_CARTESIAN + HEADER_SIZE+1]= (quint8)(x_u>>8);
    cmd[DATA_MOVE_X_CARTESIAN + HEADER_SIZE]= (quint8)(x_u);

    cmd[DATA_MOVE_Y_CARTISIAN + HEADER_SIZE+3]= (quint8)(y_u>>24);
    cmd[DATA_MOVE_Y_CARTISIAN + HEADER_SIZE+2]= (quint8)(y_u>>16);
    cmd[DATA_MOVE_Y_CARTISIAN + HEADER_SIZE+1]= (quint8)(y_u>>8);
    cmd[DATA_MOVE_Y_CARTISIAN + HEADER_SIZE]= (quint8)(y_u);

    cmd[DATA_MOVE_Z_CARTESIAN + HEADER_SIZE+3]= (quint8)(z_u>>24);
    cmd[DATA_MOVE_Z_CARTESIAN + HEADER_SIZE+2]= (quint8)(z_u>>16);
    cmd[DATA_MOVE_Z_CARTESIAN + HEADER_SIZE+1]= (quint8)(z_u>>8);
    cmd[DATA_MOVE_Z_CARTESIAN + HEADER_SIZE]= (quint8)(z_u);

    cmd[DATA_MOVE_ROLL_CARTESIAN + HEADER_SIZE+3]= (quint8)(roll_u>>24);
    cmd[DATA_MOVE_ROLL_CARTESIAN + HEADER_SIZE+2]= (quint8)(roll_u>>16);
    cmd[DATA_MOVE_ROLL_CARTESIAN + HEADER_SIZE+1]= (quint8)(roll_u>>8);
    cmd[DATA_MOVE_ROLL_CARTESIAN + HEADER_SIZE]= (quint8)(roll_u);

    cmd[DATA_MOVE_PITCH_CARTESIAN + HEADER_SIZE+3]= (quint8)(pitch_u>>24);
    cmd[DATA_MOVE_PITCH_CARTESIAN + HEADER_SIZE+2]= (quint8)(pitch_u>>16);
    cmd[DATA_MOVE_PITCH_CARTESIAN + HEADER_SIZE+1]= (quint8)(pitch_u>>8);
    cmd[DATA_MOVE_PITCH_CARTESIAN + HEADER_SIZE]= (quint8)(pitch_u);

    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE+3]= (quint8)(yaw_u>>24);
    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE+2]= (quint8)(yaw_u>>16);
    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE+1]= (quint8)(yaw_u>>8);
    cmd[DATA_MOVE_YAW_CARTESIAN+HEADER_SIZE]= (quint8)(yaw_u);

    return cmd;
}


QByteArray YRC1000micro_command::SelectJob(QString job_name)
{
    QByteArray cmd = header_to_send + job_to_send;
    cmd[CMD_DATA_SIZE] = CMD_HEADER_SELECT_JOB_DATA_SIZE;
    cmd[CMD_ID_NO]     = CMD_ID_SELECT_JOB;
    cmd[CMD_INSTANCE]  = CMD_HEADER_SELECT_JOB_INSTANCE_SET_EXE_JOB;
    cmd[CMD_ATTRIBUTE] = CMD_HEADER_SELECT_JOB_ATTRIBUTE_JOB_ALL;
    cmd[CMD_SERVICE]   = CMD_HEADER_SELECT_JOB_SERVICE_ALL;

    QByteArray name = job_name.toUtf8();

    for (int i = 0; i < job_name.size(); i++)
    {
        cmd[DATA_SELECT_JOB_NAME + i] = name[i];
    }

    /*
     * Choose line number 1 instead of 0 because code line number 0 is always "NOP"
     */
    cmd[DATA_SELECT_JOB_LINE_NUM] = '\x01';

    return cmd;
}

QByteArray YRC1000micro_command::StartJob()
{
    QByteArray cmd = header_to_send + data_to_send;
    cmd[CMD_DATA_SIZE] = CMD_HEADER_START_JOB_DATA_SIZE;
    cmd[CMD_ID_NO]     = CMD_ID_START_JOB;
    cmd[CMD_INSTANCE]  = CMD_HEADER_START_JOB_INSTANCE;
    cmd[CMD_ATTRIBUTE] = CMD_HEADER_START_JOB_ATTRIBUTE;
    cmd[CMD_SERVICE]   = CMD_HEADER_START_JOB_SERVICE;
    cmd[DATA_BYTE0]    = CMD_DATA_START_JOB;

    return cmd;
}

QByteArray YRC1000micro_command::TeachRobotPosition(quint8 coordinate_type, int index, QVector<double> *position)
{
    QByteArray cmd = header_to_send + teach_position_data_part;
    cmd[CMD_DATA_SIZE] = CMD_HEADER_TEACH_POSITION_DATA_SIZE;
    cmd[CMD_ID_NO] = CMD_ID_TEACH_ROBOT_POSITION;
    cmd[CMD_INSTANCE] = index;
    cmd[CMD_ATTRIBUTE] = CMD_HEADER_TEACH_POSITION_ATTRIBUTE_ALL;
    cmd[CMD_SERVICE] = CMD_HEADER_TEACH_POSITION_SERVICE_WRITE;

    cmd[DATA_TEACH_POSITION_DATA_TYPE] = coordinate_type;
    cmd[DATA_TEACH_POSITION_COORDINATE_NUM] = '\x01';

    quint32 axis1 = (quint32)(position->at(0)*1000);
    quint32 axis2 = (quint32)(position->at(1)*1000);
    quint32 axis3 = (quint32)(position->at(2)*1000);
    quint32 axis4 = (quint32)(position->at(3)*10000);
    quint32 axis5 = (quint32)(position->at(4)*10000);
    quint32 axis6 = (quint32)(position->at(5)*10000);

    cmd[DATA_TEACH_POSITION_COORDINATE_1 + 3] = (quint8)(axis1 >> 24);
    cmd[DATA_TEACH_POSITION_COORDINATE_1 + 2] = (quint8)(axis1 >> 16);
    cmd[DATA_TEACH_POSITION_COORDINATE_1 + 1] = (quint8)(axis1 >> 8);
    cmd[DATA_TEACH_POSITION_COORDINATE_1] = (quint8)axis1;

    cmd[DATA_TEACH_POSITION_COORDINATE_2 + 3] = (quint8)(axis2 >> 24);
    cmd[DATA_TEACH_POSITION_COORDINATE_2 + 2] = (quint8)(axis2 >> 16);
    cmd[DATA_TEACH_POSITION_COORDINATE_2 + 1] = (quint8)(axis2 >> 8);
    cmd[DATA_TEACH_POSITION_COORDINATE_2] = (quint8)axis2;

    cmd[DATA_TEACH_POSITION_COORDINATE_3 + 3] = (quint8)(axis3 >> 24);
    cmd[DATA_TEACH_POSITION_COORDINATE_3 + 2] = (quint8)(axis3 >> 16);
    cmd[DATA_TEACH_POSITION_COORDINATE_3 + 1] = (quint8)(axis3 >> 8);
    cmd[DATA_TEACH_POSITION_COORDINATE_3] = (quint8)axis3;

    cmd[DATA_TEACH_POSITION_COORDINATE_4 + 3] = (quint8)(axis4 >> 24);
    cmd[DATA_TEACH_POSITION_COORDINATE_4 + 2] = (quint8)(axis4 >> 16);
    cmd[DATA_TEACH_POSITION_COORDINATE_4 + 1] = (quint8)(axis4 >> 8);
    cmd[DATA_TEACH_POSITION_COORDINATE_4] = (quint8)axis4;

    cmd[DATA_TEACH_POSITION_COORDINATE_5 + 3] = (quint8)(axis5 >> 24);
    cmd[DATA_TEACH_POSITION_COORDINATE_5 + 2] = (quint8)(axis5 >> 16);
    cmd[DATA_TEACH_POSITION_COORDINATE_5 + 1] = (quint8)(axis5 >> 8);
    cmd[DATA_TEACH_POSITION_COORDINATE_5] = (quint8)axis5;

    cmd[DATA_TEACH_POSITION_COORDINATE_6 + 3] = (quint8)(axis6 >> 24);
    cmd[DATA_TEACH_POSITION_COORDINATE_6 + 2] = (quint8)(axis6 >> 16);
    cmd[DATA_TEACH_POSITION_COORDINATE_6 + 1] = (quint8)(axis6 >> 8);
    cmd[DATA_TEACH_POSITION_COORDINATE_6] = (quint8)axis6;
    return cmd;
}
