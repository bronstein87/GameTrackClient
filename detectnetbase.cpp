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
