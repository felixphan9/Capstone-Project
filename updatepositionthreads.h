#ifndef UPDATEPOSITIONTHREADS_H
#define UPDATEPOSITIONTHREADS_H

#include <QThread>
#include <QObject>

class updatePositionThreads : public QThread
{
public:
    explicit updatePositionThreads(QObject *parent = nullptr);
};

#endif // UPDATEPOSITIONTHREADS_H
