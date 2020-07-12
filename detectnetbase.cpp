#include "detectnetbase.h"

DetectNetBase::DetectNetBase(qint32 width, qint32 height, QObject *parent) : QObject(parent)
{
    const size_t imgSize = width * height * sizeof(float) * 4;
    qDebug() << cudaAllocMapped((void**)&imgCPU, (void**)&imgCUDA, imgSize) << "alloc mapped";
}

void DetectNetBase::initCNN(detectNet::NetworkType type, double threshold)
{
    net.reset(detectNet::Create(type, threshold));
}

void DetectNetBase::initCNN(const QString &deploy, const QString &snapshot, double threshold)
{
    net.reset(detectNet::Create(deploy.toUtf8().constData(), snapshot.toUtf8().constData(), 0.0f, NULL, threshold, "data"));
}

DetectNetBase::~DetectNetBase()
{
    CUDA(cudaFreeHost(imgCPU));
}

void DetectNetBase::readFrameInGPUMemory(Mat img)
{
    float4* cpuPtr = *(float4**)&imgCPU;

    for (qint32 y = 0; y < img.rows; y++ )
    {
        for (qint32 x = 0; x < img.cols; x++ )
        {
            quint8* p = img.ptr <quint8> (y,x);
            const float4 px = make_float4(float(p[2]),
                    float(p[1]),
                    float(p[0]), 1.0f);

            cpuPtr[y * img.cols + x] = px;
        }
    }
}

DetectNetBase::IOUResult DetectNetBase::calculateIOU(Rect2d r1, Rect2d r2)
{
    double xA = max(r1.x, r2.x);
    double yA = max(r1.y, r2.y);
    double xB = min(r1.x + r1.width, r2.x + r2.width);
    double yB = min(r1.y + r1.height, r2.y + r2.height);

    double interArea = max(0., xB - xA + 1) * max(0., yB - yA + 1);

    IOUResult res;
    res.iou = interArea / float(r1.area() + r2.area() - interArea);
    res.partR1 = interArea / r1.area();
    res.partR2 = interArea / r2.area();
    return res;
}
