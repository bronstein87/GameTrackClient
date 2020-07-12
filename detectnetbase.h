#ifndef DETECTNETBASE_H
#define DETECTNETBASE_H

#include <QObject>
#include <jetson-inference/detectNet.h>
#include <jetson-utils/cudaMappedMemory.h>
#include <jetson-utils/loadImage.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/tracking.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <QDebug>

using namespace cv;
class DetectNetBase : public QObject
{

    struct IOUResult
    {
        double iou;
        double partR1;
        double partR2;
    };


    Q_OBJECT
public:
    explicit DetectNetBase(qint32 width, qint32 height, QObject *parent = nullptr);

    void initCNN(detectNet::NetworkType type, double threshold);

    void initCNN(const QString& deploy, const QString& snapshot, double threshold);

    IOUResult calculateIOU(Rect2d r1, Rect2d r2);

    ~DetectNetBase();

signals:

protected:
    void readFrameInGPUMemory(Mat img);

    QScopedPointer <detectNet> net;
    float* imgCPU    = nullptr;
    float* imgCUDA   = nullptr;
    constexpr const static qint32 maxImageSize = 640;

private:



};

#endif // DETECTNETBASE_H

