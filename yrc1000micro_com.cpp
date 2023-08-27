#include "yrc1000micro_com.h"
#include "udp.h"

YRC1000micro_com::YRC1000micro_com(QObject *parent) : QObject(parent)
{
    request_id_index = 0;

    job_select_status_error = true;
    job_start_status_error = false;
    servo_status = false;
    /*Boolean variables for updating vars*/
    read_pos = false;

    /*Connect SIGNAL dataReceiveSignal() -> function YRC1000mcroDataCallback() */
    connect(&udp_server, SIGNAL(dataReceiveSignal()), this, SLOT(YRC1000microDataCallback()));
}

YRC1000micro_com::~YRC1000micro_com()
{
    udp_server.~UDP();
}

//Funcion for data receive handler and emit signal for UI Update SLOT
void YRC1000micro_com::YRC1000microDataCallback()
{
    QByteArray data = udp_server.getUdpData();

    quint8 res_id_index = (quint8)data[CMD_REQUEST_ID];
    char response_id = request_code[res_id_index];

    if (response_id == CMD_ID_READ_POS){
        if (read_position_type == READ_POSITION_XYZ)
        {
            YRC1000microReadPositionResponse(data);
        }
        else if (read_position_type == READ_POSITION_PULSE)
        {
            YRC1000microReadPulseResponse(data);
        }
    }
    else if(response_id == CMD_ID_STATUS_READING){
        YRC1000microReadStatusResponse(data);
    }

    else if(response_id == CMD_ID_SELECT_JOB)
    {
        YRC1000microSelectJobResponse(data);
    }

    else if(response_id == CMD_ID_TEACH_ROBOT_POSITION){
        YRC1000mircoTeachPositionResponse(data);
    }

//    Emit signal for UI to update UI form
    emit dataUIReceiveSignal(servo_status, job_start_status_error, job_select_status_error);
}


//Get the robot_position vector array for being used in other class
QVector<double> YRC1000micro_com::YRC1000microUpdateRobotPosition(){
    return robot_position;
}


//Get the address: 192.168.1.15 and 10040 port number store in udp_address and udp_port
void YRC1000micro_com::YRC1000microSetConnection(QHostAddress address,quint16 port,quint16 file_port)
{
    qDebug() << "Set connection ";
    udp_address = address;
    udp_port = port;
    udp_file_port = file_port;
}


bool YRC1000micro_com::YRC1000microConnect()
{
    bool connect_satus = udp_server.udpConnect(udp_address, udp_port);
    return connect_satus;
}


//UDP Socket disconnect function
void YRC1000micro_com::YRC1000microDisConnect()
{
    udp_server.udpDisConnect();
}


void YRC1000micro_com::YRC1000microReadStatusResponse(QByteArray data)
{
    running_status = 0;
    running_status = (quint8)data[HEADER_SIZE];
//    qDebug() << "running_status 1: " << (quint32)(quint8)data[HEADER_SIZE];
//    qDebug() << "Robot Status 2: " << (quint8)data[HEADER_SIZE + 1];
//    qDebug() << "Robot Status 3: " << (quint8)data[HEADER_SIZE + 2];
//    qDebug() << "Robot Status 4: " << (quint8)data[HEADER_SIZE + 3];
    running_status = running_status | (quint8)data[DATA_BYTE0];
    running_status &= RESPONSE_VALUE_READ_STATUS_MASK;
}

//Function for reading robot status, send the header command and wait for receiver
// for every timer tick
void YRC1000micro_com::YRC1000microReadStatus()
{
    cmd = yrc1000micro_command.ReadStatus();
    udp_server.sendData(udp_address, udp_port, cmd);

    //Save the request_code
    request_code[request_id_index] = CMD_ID_STATUS_READING;

    //Incre by 1 for postcommands
    request_id_index++;
    //Clear the data after finish
    cmd.clear();
}

quint8 YRC1000micro_com::YRC1000microUpdateStatus()
{
     return running_status;
}

void YRC1000micro_com::YRC1000microOnServo()
{
//    Get the OnServo Command
//    yrc1000micro_command.OnServo();

    cmd = yrc1000micro_command.OnServo();
    udp_server.sendData(udp_address, udp_port, cmd);

    //Save the request_code
    request_code[request_id_index] = CMD_ID_SERVO_ON;

    //Incre by 1 for postcommands
    request_id_index++;
    //Clear the data after finish
    cmd.clear();
}

void YRC1000micro_com::YRC1000microOffServo()
{
    cmd = yrc1000micro_command.OffServo();

    //Change the Data part -> 0x02 for OFF command
    cmd[CMD_REQUEST_ID] = request_id_index;

    //Send the command
    udp_server.sendData(udp_address, udp_port, cmd);

    //Save the request_code
    request_code[request_id_index] = CMD_ID_SERVO_ON;

    //Incre by 1 for postcommands
    request_id_index++;
    //Clear the data after finish
    cmd.clear();
}


void YRC1000micro_com::YRC1000microMoveCartesian(quint8 coordinate,quint8 move_type, quint8 speed_type,
                                                 double speed, QVector<double>* position)
{
//    qDebug() << "Move in Cartesian Coordinate!" ;
    cmd = yrc1000micro_command.setRobotPositionCartesian(coordinate, move_type,
                                                  speed_type, speed, position);

    cmd[CMD_REQUEST_ID] = request_id_index;

    udp_server.sendData(udp_address,udp_port, cmd);

    request_code[request_id_index] = CMD_ID_MOVE_ROBOT_CARTESIAN;

    cmd.clear();
    request_id_index ++;
}


void YRC1000micro_com::YRC1000microReadPositionResponse(QByteArray data)
{
    qint32 x_axis = 0;
    qint32 y_axis = 0;
    qint32 z_axis = 0;
    qint32 roll_angle = 0;
    qint32 pitch_angle = 0;
    qint32 yaw_angle = 0;

//Shift bit to get the 32 bit integer value.
    x_axis = ((quint32)(quint8)data[DATA_X_AXIS+3])<<24 | ((quint32)(quint8)data[DATA_X_AXIS+2])<<16 |
                    ((quint32)(quint8)data[DATA_X_AXIS+1])<<8 | (quint32)(quint8)data[DATA_X_AXIS];

    y_axis = ((quint32)(quint8)data[DATA_Y_AXIS+3])<<24 | ((quint32)(quint8)data[DATA_Y_AXIS+2])<<16 |
                    ((quint32)(quint8)data[DATA_Y_AXIS+1])<<8 | (quint32)(quint8)data[DATA_Y_AXIS];

    z_axis = ((quint32)(quint8)data[DATA_Z_AXIS+3])<<24 | ((quint32)(quint8)data[DATA_Z_AXIS+2])<<16 |
                    ((quint32)(quint8)data[DATA_Z_AXIS+1])<<8 | (quint32)(quint8)data[DATA_Z_AXIS];

    roll_angle = ((quint32)(quint8)data[DATA_ROLL_ANGLE+3])<<24 | ((quint32)(quint8)data[DATA_ROLL_ANGLE+2])<<16 |
                    ((quint32)(quint8)data[DATA_ROLL_ANGLE+1])<<8 | (quint32)(quint8)data[DATA_ROLL_ANGLE];

    pitch_angle = ((quint32)(quint8)data[DATA_PITCH_ANGLE+3])<<24 | ((quint32)(quint8)data[DATA_PITCH_ANGLE+2])<<16 |
                    ((quint32)(quint8)data[DATA_PITCH_ANGLE+1])<<8 | (quint32)(quint8)data[DATA_PITCH_ANGLE];

    yaw_angle = ((quint32)(quint8)data[DATA_YAW_ANGLE+3])<<24 | ((quint32)(quint8)data[DATA_YAW_ANGLE+2])<<16 |
                    ((quint32)(quint8)data[DATA_YAW_ANGLE+1])<<8 | (quint32)(quint8)data[DATA_YAW_ANGLE];

//    qDebug() << "X Axis : " << x_axis ;
//    qDebug() << "Y Axis : " << y_axis ;
//    qDebug() << "Z Axis : " << z_axis ;
//    qDebug() << "Roll Axis : " << roll_angle ;
//    qDebug() << "Pitch Axis : " << pitch_angle ;
//    qDebug() << "Yaw Axis : " << yaw_angle ;

//    Save into vector robot_position
    robot_position[0]=(x_axis/1000.0);
    robot_position[1]=(y_axis/1000.0);
    robot_position[2]=(z_axis/1000.0);

    robot_position[3]=(roll_angle/10000.0);
    robot_position[4]=(pitch_angle/10000.0);
    robot_position[5]=(yaw_angle/10000.0);
}

//This function send request message to Robot and wait for response to read position in XYZ
void YRC1000micro_com::YRC1000microReadPosition()
{
    cmd = yrc1000micro_command.ReadPosition(); //Just send the header for command
    cmd[CMD_REQUEST_ID] = request_id_index;

    /*Send the command*/
    udp_server.sendData(udp_address, udp_port, cmd);

    //Save the request_code according to index
    request_code[request_id_index] = CMD_ID_READ_POS;
    read_position_type = READ_POSITION_XYZ;
    cmd.clear();
    //Incre by 1 for postcommands
    request_id_index++;

}

void YRC1000micro_com::YRC1000microReadPulse()
{
    cmd = yrc1000micro_command.ReadPulse(); //Just send the header for command
    cmd[CMD_REQUEST_ID] = request_id_index;

    /*Send the command*/
    udp_server.sendData(udp_address, udp_port, cmd);

    //Save the request_code according to index
    request_code[request_id_index] = CMD_ID_READ_POS;

    read_position_type = READ_POSITION_PULSE;
    cmd.clear();
    //Incre by 1 for postcommands
    request_id_index++;
}

void YRC1000micro_com::YRC1000microReadPulseResponse(QByteArray data)
{
    qint32 j1 = 0;
    qint32 j2 = 0;
    qint32 j3 = 0;
    qint32 j4 = 0;
    qint32 j5 = 0;
    qint32 j6 = 0;
    j1 = ((quint32)(quint8)data[DATA_X_AXIS+3])<<24 | ((quint32)(quint8)data[DATA_X_AXIS+2])<<16 |
                    ((quint32)(quint8)data[DATA_X_AXIS+1])<<8 | (quint32)(quint8)data[DATA_X_AXIS];

    j2 = ((quint32)(quint8)data[DATA_Y_AXIS+3])<<24 | ((quint32)(quint8)data[DATA_Y_AXIS+2])<<16 |
                    ((quint32)(quint8)data[DATA_Y_AXIS+1])<<8 | (quint32)(quint8)data[DATA_Y_AXIS];

    j3 = ((quint32)(quint8)data[DATA_Z_AXIS+3])<<24 | ((quint32)(quint8)data[DATA_Z_AXIS+2])<<16 |
                    ((quint32)(quint8)data[DATA_Z_AXIS+1])<<8 | (quint32)(quint8)data[DATA_Z_AXIS];

    j4 = ((quint32)(quint8)data[DATA_ROLL_ANGLE+3])<<24 | ((quint32)(quint8)data[DATA_ROLL_ANGLE+2])<<16 |
                    ((quint32)(quint8)data[DATA_ROLL_ANGLE+1])<<8 | (quint32)(quint8)data[DATA_ROLL_ANGLE];

    j5 = ((quint32)(quint8)data[DATA_PITCH_ANGLE+3])<<24 | ((quint32)(quint8)data[DATA_PITCH_ANGLE+2])<<16 |
                    ((quint32)(quint8)data[DATA_PITCH_ANGLE+1])<<8 | (quint32)(quint8)data[DATA_PITCH_ANGLE];

    j6 = ((quint32)(quint8)data[DATA_YAW_ANGLE+3])<<24 | ((quint32)(quint8)data[DATA_YAW_ANGLE+2])<<16 |
                    ((quint32)(quint8)data[DATA_YAW_ANGLE+1])<<8 | (quint32)(quint8)data[DATA_YAW_ANGLE];

//    QVector<double> pos;
    robot_pulse[0]= j1;
    robot_pulse[1]= j2;
    robot_pulse[2]= j3;
    robot_pulse[3]= j4;
    robot_pulse[4]= j5;
    robot_pulse[5]= j6;
}

QVector<double> YRC1000micro_com::YRC1000microupdateRobotPulse(){
    return robot_pulse;
}


void YRC1000micro_com::forwardKinematic(QVector<double> *theta, QVector<double> *output_cartesian)
{
    QVector<double> a(6, 0);
    QVector<double> d(6, 0);
    QVector<double> cv_theta(6, 0);
    QVector<double> alpha(6, 0);

    a[0] =   20;
    a[1] = -165;
    d[0] =    0;
    d[3] = -165;
    d[5] =  -40;

    cv_theta[0] = theta->at(0);
    cv_theta[1] = theta->at(1) + M_PI/2;
    cv_theta[2] = theta->at(2) - M_PI;
    cv_theta[3] = theta->at(3);
    cv_theta[4] = theta->at(4) - M_PI/2;
    cv_theta[5] = theta->at(5);

    alpha[0] = -M_PI/2;
    alpha[1] = M_PI;
    alpha[2] = -M_PI/2;
    alpha[3] = M_PI/2;
    alpha[4] = -M_PI/2;
    alpha[5] = M_PI;

    QVector<QMatrix4x4> A(6);
    for(int i = 0; i <= 5; i++)
    {
        A[i] = QMatrix4x4(cos(cv_theta[i]) , -sin(cv_theta[i])*cos(alpha[i]) , sin(cv_theta[i])*sin(alpha[i])  , a[i]*cos(cv_theta[i]) ,
                          sin(cv_theta[i]) , cos(cv_theta[i])*cos(alpha[i])  , -cos(cv_theta[i])*sin(alpha[i]) , a[i]*sin(cv_theta[i]) ,
                          0                , sin(alpha[i])                   , cos(alpha[i])                   , d[i]                ,
                          0                , 0                               , 0                               , 1);
    }

    bTe = A[0]*A[1]*A[2]*A[3]*A[4]*A[5];
    (*output_cartesian)[0] = bTe(0, 3);
    (*output_cartesian)[1] = bTe(1, 3);
    (*output_cartesian)[2] = bTe(2, 3);

    double sy = sqrt(pow(bTe(0,0), 2) + pow(bTe(1, 0), 2));
    if(sy >= 1e-6){
        (*output_cartesian)[3] = qAtan2(bTe(2,1),bTe(2,2));
        (*output_cartesian)[4] = qAtan2(-bTe(2,0),sy);
        (*output_cartesian)[5] = qAtan2(bTe(1,0),bTe(0,0));
    }else{
        (*output_cartesian)[3] = qAtan2(-bTe(1,2),bTe(1,1));
        (*output_cartesian)[4] = qAtan2(-bTe(2,0),sy);
        (*output_cartesian)[5] = 0;
    }
}

//---Inverse Kinematics
void YRC1000micro_com::inverseKinematic(QVector<double> *pos, QVector<double> *output_deg)
{
    //Calculate theta-1

}
QMatrix4x4 YRC1000micro_com::getbTe()
{
    return bTe;
}
void YRC1000micro_com::YRC1000microSelectJob(QString job_name)
{
    cmd = yrc1000micro_command.SelectJob(job_name);
    cmd[CMD_REQUEST_ID] = request_id_index;

    request_code[request_id_index] = CMD_ID_SELECT_JOB;

    /*Send the command*/
    udp_server.sendData(udp_address, udp_port, cmd);

    cmd.clear();
    //Incre by 1 for postcommands
    request_id_index++;

    job = job_name;
}

void YRC1000micro_com::YRC1000microSelectJobResponse(QByteArray data)
{
    if(data[CMD_STATUS] != '\x00')
    {
        qDebug() << "Failed to select job";
        job_select_status_error = true;
        return;
    }
    else
    {
        job_select_status_error = false;
        qDebug() << "Job selected: " + job;
        emit selectJobSignal(job);
    }
}

void YRC1000micro_com::YRC1000microStartJob()
{
    if( !job_select_status_error )
    {
        cmd = yrc1000micro_command.StartJob();
        cmd[CMD_REQUEST_ID] = request_id_index;

        request_code[request_id_index] = CMD_ID_START_JOB;
        /*Send the command*/
        udp_server.sendData(udp_address, udp_port, cmd);

        cmd.clear();
        request_id_index++;
    }
    else
    {
        qDebug() << "Failed to start job";
        job_start_status_error = true;
    }
}

void YRC1000micro_com::YRC1000mircoTeachPosition(quint8 coordinate_type, int index, QVector<double> *position)
{
    cmd = yrc1000micro_command.TeachRobotPosition(coordinate_type, index, position);
    udp_server.sendData(udp_address, udp_port, cmd);
    cmd[CMD_REQUEST_ID] = request_id_index;
    request_code[request_id_index] = CMD_ID_TEACH_ROBOT_POSITION;
    cmd.clear();
    request_id_index++;
}

void YRC1000micro_com::YRC1000mircoTeachPositionResponse(QByteArray data)
{
    if(data[CMD_STATUS] != '\x00')
    {
        qDebug() << "Failed to teach position";
        teach_signal_error = 1;
        return;
    }
    else {
        qDebug() << "Position is taught";
    }
}
//=======================================================================================================================
//-----------------------Start Soldering --------------------------------------------------------------------------------
void YRC1000micro_com::startSoldering(uint count, std::vector<std::vector<double>> XY_vec, double Z , QVector<double> homePosition)
{
    //---Which thread is this running in
    qInfo() << QThread::currentThread();
    //---Variables-----
    QVector<double> solderingPosition = QVector<double> (6);
    double set_speed = 40.0;
    double nextZ = Z + 10.0;
    //---Get Pose------
    solderingPosition = homePosition;

    //B1: ---Move back to home position---
    YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                               CMD_DATA_MOVE_SPEED_TYPE_LINK, set_speed, &homePosition);
    usleep(10*microsecond);
    set_speed = 5.0;

    //B2: ---Loop--- XY_vec[i][0] -> X; XY_vec[i][1] -> Y
    for(uint i = 0; i < count; i++)
    {
        std::cout << "=======================" << std::endl;
        std::cout << "Soldering pad index: " << i + 1 << " ...." << std::endl;
        std::cout << "X-Solder: " << XY_vec[i][0] << std::endl;
        std::cout << "Y-Solder: " << XY_vec[i][1] << std::endl;
        if(i == 0) // with 10mm/s
        {
            // ---First Pad - Move up first wait 10.0s
            set_speed = 15.0;
            solderingPosition[0] = XY_vec[i][0];
            solderingPosition[1] = XY_vec[i][1];
            YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                   CMD_DATA_MOVE_SPEED_TYPE_LINK, set_speed, &solderingPosition);
            usleep(5.0*microsecond);
//            emit motorRun(); //Feed some flux for better soldering
            usleep(5.0*microsecond);
            //--- Solder first Pad
            solderingPosition[0] = XY_vec[i][0];
            solderingPosition[1] = XY_vec[i][1];
            solderingPosition[2] = Z;

            YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                   CMD_DATA_MOVE_SPEED_TYPE_LINK, set_speed, &solderingPosition);

            usleep(3*microsecond);
        }
        else // with 5mm/s
        {
            solderingPosition[0] = XY_vec[i][0];
            solderingPosition[1] = XY_vec[i][1];
            solderingPosition[2] = nextZ;
            YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                   CMD_DATA_MOVE_SPEED_TYPE_LINK, set_speed, &solderingPosition);
            usleep(5*microsecond);
            //--- Solder
            solderingPosition[2] = Z;
            YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                                   CMD_DATA_MOVE_SPEED_TYPE_LINK, set_speed, &solderingPosition);
            //--- Go down to Solder Position wait 5second
            usleep(3*microsecond);
        }
        //--Feed wire then wait 3s
        emit motorRun();
        usleep(4*microsecond);

    //B3: --Move up Z for next pad wait 1s
        solderingPosition[2] = nextZ;
        YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                               CMD_DATA_MOVE_SPEED_TYPE_LINK, set_speed, &solderingPosition);
        usleep(1*microsecond);
    }
    //B4: --Move back to home with 30 mm/s
    set_speed = 30.0;
    YRC1000microMoveCartesian(CMD_DATA_MOVE_COORDINATE_ROBOT, CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE,
                                               CMD_DATA_MOVE_SPEED_TYPE_LINK, set_speed, &homePosition);
}

