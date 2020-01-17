#ifndef MAINSTRUCTS
#define MAINSTRUCTS
#include <QtGlobal>
#include <opencv2/core.hpp>
#include <imageprocext.h>
using namespace cv;
using namespace imageprocext;

static const constexpr int defaultHSV = -256;
struct FrameTime
{
    cv::Mat frame;
    quint64 time = 0;
};


enum LightningParameter
{
    Invalid,
    Day,
    Evening
};


#pragma pack(push, 1)
struct RecROIs
{
    cv::Rect mainSearchRect;
    cv::Rect trackFirstRect;
    cv::Rect trackSecondRect;
    cv::Rect mainHitSearchRect;
};
#pragma pack(pop)



#pragma pack(push, 1)
struct RecognizeParameters
{
    double corrCoef = 0.8;
    double skoCoef = 14; // mean + coef * sko for threshold for substracted frame
    qint32 searchAreaSize = 100; // search area for already tracked ball (prev_center - searchAreaSize, prev_center + searchAreaSize)
    double minSkoOnTemplate = 2.5;
    double maxAngleBetwDirections = 25; //degrees
    double minSpeed = 5; // pixel
    qint32 cannyThresMin = 50;
    qint32 cannyThresMax = 150;
    qint32 maxArea = 25;
    qint32 minArea = 6;
    double circularityCoeff = 0.8;
    double dirX = -1;
    double dirY = -1;
};
#pragma pack(pop)


#pragma pack(push, 1)
struct CameraOptions
{
    qint32 pictureParamFlag = 0;
    qint32 whiteBalance = 0;
    qint32 equalization = 0;
    qint32 autoExposureIntenal = 0;
    qint32 focusing = -1;
    double exposure = 1;
    double frameRate = 60;
    double frameRateSend = 30;
    qint32 pixelClock = 474;
    double gamma = -1;
    qint32 gain = -1;
    double sharp = -1;
    qint32 AOIHeight = 1080;
    qint32 AOIWidth = 1920;
    qint32 portSendStream = -1;
    qint32 IRCorrection = 0;
    qint32 saturation = defaultHSV;
    qint32 rSaturation = defaultHSV;
    qint32 gSaturation = defaultHSV;
    qint32 bSaturation = defaultHSV;
    qint32 hue = defaultHSV;
    double shadowCoef = -1;
    double shadowThreshold = -1;
    qint32 shadowGaussWindowSize = -1;
    RecROIs recRois;
    Rect wbRect; // tmp
    AutoExposureHandler::AutoExposureParameters autoExpParams;
    qint32 rawFrame = -1;
    qint32 videoDuration = 5;
    qint32 triggerMode = 0;
    RecognizeParameters recParams;
    qint32 ballRecognizeFlag = 0;
    qint32 ballRecognizeStep = 3;
    qint32 debugRecFlag = 0;
    qint32 debounceEnable = 0;
    qint32 debounceValue = 200;
    qint32 rotate = 0;

};
#pragma pack(pop)

#pragma pack(push, 1)
struct CurrentCameraParams
{
    double exposure;
    double minExposure;
    double maxExposure;
    qint32 pixelClock;
    qint32 minPixelClock;
    qint32 maxPixelClock;
    double frameRate;
    double minFrameRate;
    double maxFrameRate;
    qint32 sendFrameRate;
    qint32 gain;
    RecROIs recRois;
    Rect wbRect; // tmp
    AutoExposureHandler::AutoExposureParameters autoExpParams;
    qint32 rawFrame = 0;
    qint32 videoDuration;
    qint32 width;
    qint32 height;
    qint32 triggerMode = 0;
    qint32 pictureParamFlag = 0;
    qint32 whiteBalance = 0;
    qint32 equalization = 0;
    qint32 autoExposureIntenal = 0;
    qint32 focusing = -1;
    double gamma = -1;
    double sharp = -1;
    qint32 portSendStream = -1;
    qint32 IRCorrection = 0;
    qint32 saturation = defaultHSV;
    qint32 rSaturation = defaultHSV;
    qint32 gSaturation = defaultHSV;
    qint32 bSaturation = defaultHSV;
    qint32 hue = defaultHSV;
    double shadowCoef = -1;
    double shadowThreshold = -1;
    qint32 shadowGaussWindowSize = -1;
    RecognizeParameters recParams;
    qint32 ballRecognizeFlag = 0;
    qint32 ballRecognizeStep = 3;
    qint32 debugRecFlag = 0;
    qint32 debounceEnable = -1;
    qint32 debounceValue = -1;
    qint32 rotate = 0;
};
#pragma pack(pop)
#endif // MAINSTRUCTS

