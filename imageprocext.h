#ifndef IMAGEPROCEXT
#define IMAGEPROCEXT

#include <opencv2/cudaimgproc.hpp>
#include <opencv2/photo/photo.hpp>
#include <QObject>
#include <QString>
using namespace cv;


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

    enum LightningParameter
    {
        Invalid,
        Day,
        Evening
    };

    #pragma pack(push, 1)
    struct AutoExposureParameters
    {
        LightningParameter light = Day;
        double gain = 30;
        double exposure = 1;
        double minGainCoeff = 30;
        double maxGainCoeff= 80;
        double maxPercent = 0.05;
        double minRelCoef = 0.9;
        double maxRelCoef = 1.1;
    };
    #pragma pack(pop)





    explicit AutoExposureHandler(QObject *parent = nullptr);

    void setOptions(const AutoExposureParameters& params);
    bool correct(Mat image);
    AutoExposureParameters params;
    double percent;
    constexpr const static double lowExp = 1.0;
    constexpr const static double maxExp = 3.0;
    const int divideCoeffDefault = 8;
    int divideCoeff = 8;
    const int divideCoeffMax = 24;
    const int maxFrameCoeff = 20;
    int processCounter = 0;

};

}



#endif // IMAGEPROCEXT

