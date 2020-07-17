#ifndef IMAGEPROCEXT
#define IMAGEPROCEXT

#include <opencv2/cudaimgproc.hpp>
#include <opencv2/photo/photo.hpp>
#include <QObject>
#include <QString>
#include <proto/msg.internal.pb.h>
#include <QDebug>
using namespace cv;
using namespace gt::internal;

// ВО ВСЕХ ФУНКЦИЯХ СДЕЛАТ БЕЗ split (вообще без копирования), обращаться к пикселям в исходном изображении
namespace imageprocext
{
Mat unsharpMasking(const Mat& frame, double coeff);

Mat gammaCorrection(const Mat& frame, double coeff);

Mat gain(const Mat &frame, double coeff, double add = 0.0);

Mat gainChannel(int channel, const Mat &frame, double coeff, double add = 0.0);

Mat equalizeHistogramm(const Mat& frame);

Mat shadowing(const Mat& frame, int thres, int windowSize, double coeff);

class AutoExposureHandler : public QObject
{
    Q_OBJECT
    
signals:
    void currentStateReady(const QString& msg);
    
public:
    
    
    explicit AutoExposureHandler(QObject *parent = nullptr);
    
    bool correct(Mat image);
    
    void setParameters(msg::AutoExposureParameters* _params)
    {
        //qDebug() <<"MEAN" << params->mean();
        params = _params;
        if (!params->has_mean())
        {
            params->set_mean(maxMean);
        }
    }
    
    msg::AutoExposureParameters* getParameters() 
    {
        return params;
    }
    
private:
    
    
    msg::AutoExposureParameters* params;
    
    double percent;
    constexpr const static double lowExp = 1.0;
    constexpr const static double maxExp = 3.0;
    constexpr const static double maxMean = 105;
    const int divideCoeffDefault = 2;
    int divideCoeff = 2;
    const int divideCoeffMax = 24;
    const int maxFrameCoeff = 20;
    int processCounter = 0;
    
};

}



#endif // IMAGEPROCEXT

