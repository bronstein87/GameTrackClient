#ifndef BATTRACKER_H
#define BATTRACKER_H

#include <QObject>
#include <detectnetbase.h>
#include <ballrecognizerpp.h>
#include <proto/msg.internal.pb.h>
#include <opencv2/features2d.hpp>

using namespace gt::internal;
class BatTracker : public DetectNetBase
{
    Q_OBJECT


public:

    enum BatTrackerStatus
    {
        Invalid,
        WaitForBall,
        BallFound,
        RecognizeBat
    };

    explicit BatTracker(qint32 width, qint32 height, QObject *parent = nullptr);

    BatTrackerStatus handle(Mat frame, qint64 time);

    void setBallRecognizer(BallRecognizerPP* _ballRecognizer);

signals:
    void batRecognized();

private:
    BallRecognizerPP* ballRecognizer;
    BatTrackerStatus status = Invalid;
    constexpr const static qint32 recognizeBatFor = 60;
    qint32 recognizeBatCount = 0;
    qint32 confirmedCount = 0;
    bool previousRecognized = false;
    QVector <QPair <cv::Rect, Mat> > recognizedROIs;
};

#endif // BATTRACKER_H
