#include "udp.h"

UDP::UDP(QObject *parent) : QObject(parent)
{
    // create a QUDP socket
    socket = new QUdpSocket(this);

    // The most common way to use QUdpSocket class is
    // to bind to an address and port using bind()
    // bool QAbstractSocket::bind(const QHostAddress & address,
    // quint16 port = 0, BindMode mode = DefaultForPlatform)
}

UDP::~UDP()
{
    socket->close();
}

bool UDP::udpConnect(QHostAddress address, quint16 port)
{
    bool status = socket->bind(QHostAddress::Any, port);

//  bool status = socket->bind(address, port);

    qDebug() << "Connect status: " << status;

//  Make a connection for signal readRead() this signal is turn on when receiving data -> SLOT readyRead()
    connect(socket, SIGNAL(readyRead()), this, SLOT(readyRead()));
    return status;
}

void UDP::udpDisConnect(){
    socket->close();
}

//Read the incoming data and emit signal for further processing.
void UDP::readyRead()
{
    // when data comes in
    // QByteArray buffer;
    buffer.resize(socket->pendingDatagramSize());

    QHostAddress sender;
    quint16 senderPort;

    // qint64 QUdpSocket::readDatagram(char * data, qint64 maxSize,
    //                 QHostAddress * address = 0, quint16 * port = 0)
    // Receives a datagram no larger than maxSize bytes and stores it in data.
    // The sender's host address and port is stored in *address and *port
    // (unless the pointers are 0).

    socket->readDatagram(buffer.data(), buffer.size(),
                         &sender, &senderPort);

//    qDebug() << "Size" << buffer.size() ;
//    qDebug() << "Message from: " << sender.toString();
//    qDebug() << "Message port: " << senderPort;
//    qDebug() << "Message: " << buffer;
    emit dataReceiveSignal();
//  return buffer;
}


void UDP::sendData(QHostAddress address, quint16 port, QByteArray data)
{
    result = socket->writeDatagram(data, address, port);
//  result = socket->writeDatagram(data, QHostAddress("192.168.1.15"), 10040);

//    qDebug() << "Command sending... " << result;
//    if(result)
//    {
//        qDebug() << "CMD PASS";
//    }
//    else
//    {
//        qDebug() << "CMD FAIL";
//    }
}

//Return data buffer
QByteArray UDP::getUdpData()
{
    return buffer;
}
