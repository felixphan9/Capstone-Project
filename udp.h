#ifndef UDP_H
#define UDP_H
#include <QObject>
#include <QUdpSocket>
#include <QDebug>

class UDP : public QObject
{
    Q_OBJECT

signals:
    void dataReceiveSignal();

public slots:
    void readyRead();

public:
    explicit UDP(QObject *parent = nullptr);
    ~UDP();

    bool udpConnect(QHostAddress address,quint16 port);
    void udpDisConnect();
    void sendData(QHostAddress address, quint16 port, QByteArray data);
    QByteArray getUdpData(); //Get the data buffer, used in other class file.

private:
    QUdpSocket *socket = nullptr;
    QByteArray buffer;

    bool result;
};

#endif // UDP_H
