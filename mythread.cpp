#include "mythread.h"

myThread::myThread(QObject *parent)
    : QThread{parent}
{

}

void myThread::run()
{
   qInfo() << "In the thread running";
}
