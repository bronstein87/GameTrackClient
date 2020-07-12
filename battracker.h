#ifndef BATTRACKER_H
#define BATTRACKER_H

#include <QObject>
#include <detectnetbase.h>

class BatTracker : public DetectNetBase
{
    Q_OBJECT
public:
    explicit BatTracker(qint32 width, qint32 height, QObject *parent = nullptr);

signals:

public slots:
};

#endif // BATTRACKER_H
