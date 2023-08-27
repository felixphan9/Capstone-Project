#ifndef YRC1000MICRO_COMMAND_H
#define YRC1000MICRO_COMMAND_H

#include <QObject>

#define HEADER_SIZE             32
#define READ_POSITION_XYZ        1
#define READ_POSITION_PULSE      0
/*Data for read position in Carthesian Coord robot */
#define DATA_X_AXIS             52
#define DATA_Y_AXIS             56
#define DATA_Z_AXIS             60
#define DATA_ROLL_ANGLE         64
#define DATA_PITCH_ANGLE        68
#define DATA_YAW_ANGLE          72

/*Move instruction command in Cartesian coordinate*/
#define CMD_DATA_MOVE_COORDINATE_ROBOT          '\x11'

#define CMD_HEADER_MOVE_INSTANCE_LINK_ABSOLUTE  '\x01'
#define CMD_HEADER_MOVE_INSTANCE_LINE_ABSOLUTE  '\x02'
#define CMD_HEADER_MOVE_INSTANCE_LINE_INCREMENT '\x03'

#define CMD_HEADER_MOVE_CARTESIAN_DATA_SIZE     '\x68'
#define CMD_HEADER_MOVE_ATTRIBUTE               '\x01'
#define CMD_HEADER_MOVE_SERVICE_ALL             '\x02'

#define CMD_DATA_MOVE_SPEED_TYPE_LINK           '\x00'
#define CMD_DATA_MOVE_SPEED_TYPE_V_SPEED        '\x01'
#define CMD_DATA_MOVE_SPEED_TYPE_VR_SPEED       '\x02'

//Config data part
#define DATA_MOVE_COORDINATE                      16
#define DATA_MOVE_SPEED_TYPE                       8
#define DATA_MOVE_SPEED                           12
#define DATA_MOVE_COORDINATE                      16
#define DATA_MOVE_X_CARTESIAN                     20
#define DATA_MOVE_Y_CARTISIAN                     24
#define DATA_MOVE_Z_CARTESIAN                     28
#define DATA_MOVE_ROLL_CARTESIAN                  32
#define DATA_MOVE_PITCH_CARTESIAN                 36
#define DATA_MOVE_YAW_CARTESIAN                   40
#define DATA_MOVE_USER_COORDINATE                 64
/*--------------------------------------------------*/
/*Header index */
#define CMD_DATA_SIZE            6
#define CMD_INSTANCE            26
#define CMD_ATTRIBUTE           28
#define CMD_SERVICE             29
#define CMD_REQUEST_ID          11

/*Data part index */
#define DATA_BYTE0              32
#define DATA_BYTE1              33
#define DATA_BYTE2              34
#define DATA_BYTE3              35

/*Command No. Index*/
#define CMD_ID_NO               24

/*Define command: */
#define CMD_ID_READ_POS                 0x75
#define CMD_ID_SERVO_ON                 0x83
#define CMD_ID_MOVE_ROBOT_CARTESIAN     0x8A
#define CMD_ID_STATUS_READING           0x72
#define CMD_STATUS                        25
#define RESPONSE_VALUE_READ_STATUS_MASK 0x08  // Running status on the bit 3
/*Servo On Off command*/
#define CMD_DATA_SERVO_ON       0x01
#define CMD_DATA_SERVO_OFF      0x02

/*Job select command*/
#define CMD_HEADER_SELECT_JOB_DATA_SIZE             '\x24'
#define CMD_ID_SELECT_JOB                           '\x87'
#define CMD_HEADER_SELECT_JOB_INSTANCE_SET_EXE_JOB  '\x01'
#define CMD_HEADER_SELECT_JOB_ATTRIBUTE_JOB_ALL     '\x00'
#define CMD_HEADER_SELECT_JOB_SERVICE_ALL           '\x02'
#define DATA_SELECT_JOB_NAME                          32
#define DATA_SELECT_JOB_LINE_NUM                      64

/*Job start command*/
#define CMD_HEADER_START_JOB_DATA_SIZE              '\x04'
#define CMD_ID_START_JOB                            '\x86'
#define CMD_HEADER_START_JOB_INSTANCE               '\x01'
#define CMD_HEADER_START_JOB_ATTRIBUTE              '\x01'
#define CMD_HEADER_START_JOB_SERVICE                '\x10'
#define CMD_DATA_START_JOB                          '\x01'


/*Teach position command*/
 #define CMD_HEADER_TEACH_POSITION_DATA_SIZE        '\x34'
 #define CMD_ID_TEACH_ROBOT_POSITION                '\x7F'
 #define CMD_HEADER_TEACH_POSITION_ATTRIBUTE_ALL    '\x00'
 #define CMD_HEADER_TEACH_POSITION_SERVICE_WRITE    '\x02'

#define DATA_TEACH_POSITION_DATA_TYPE                 32
#define DATA_TEACH_POSITION_COORDINATE_NUM            44
#define DATA_TEACH_POSITION_COORDINATE_1              52
#define DATA_TEACH_POSITION_COORDINATE_2              56
#define DATA_TEACH_POSITION_COORDINATE_3              60
#define DATA_TEACH_POSITION_COORDINATE_4              64
#define DATA_TEACH_POSITION_COORDINATE_5              68
#define DATA_TEACH_POSITION_COORDINATE_6              72

/*Type data */
#define CMD_DATA_TEACH_POSITION_COORDINATE_TYPE_PULSE       '\x00'
#define CMD_DATA_TEACH_POSITION_COORDINATE_TYPE_BASE        '\x10'
#define CMD_DATA_TEACH_POSITION_COORDINATE_TYPE_ROBOT       '\x11'
#define CMD_DATA_TEACH_POSITION_COORDINATE_TYPE_USER        '\x12'
#define CMD_DATA_TEACH_POSITION_COORDINATE_TYPE_TOOL        '\x13'

class YRC1000micro_command : public  QObject
{
    Q_OBJECT

public:
    explicit YRC1000micro_command(QObject *parent = nullptr);
    ~YRC1000micro_command();

    QByteArray OnServo();
    QByteArray OffServo();
    QByteArray ReadPosition();
    QByteArray ReadStatus();
    QByteArray setRobotPositionCartesian(quint8 coordinate, quint8 move_type,
                                         quint8 speed_type, double speed, QVector<double>* position);
    QByteArray ReadPulse();

    QByteArray SelectJob(QString);
    QByteArray StartJob();

    QByteArray TeachRobotPosition(quint8 coordinate_type, int index, QVector<double> *position);
signals:

public slots:

private:

    QByteArray header_to_send;
    QByteArray data_to_send;
    QByteArray job_to_send;
    QByteArray move_cartesian_data;
    QByteArray teach_position_data_part;
};

#endif // YRC1000MICRO_COMMAND_H
