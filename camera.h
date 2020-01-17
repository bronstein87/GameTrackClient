#ifndef CAMERA_H
#define CAMERA_H

#include <QObject>
#include <QAtomicInteger>
#include <ueye.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudacodec.hpp>
#include <opencv2/xphoto/white_balance.hpp>
#include <QVector>
#include <QLinkedList>
#include <objectivecontroller.h>
#include <QDebug>
#include <imageprocext.h>
#include <QMutex>
#include <QMutexLocker>
#include <QSettings>
#include <ballrecognizer.h>
#include <QtConcurrent/QtConcurrent>
#include <mainstructs.h>
#include <mathfunc.h>
using namespace BOKZMath;




using namespace imageprocext;

struct VideoToSend
{
    QLinkedList <FrameTime> video;
    qint32 leftToAppend = 60;
};

class Camera : public QObject
{
    Q_OBJECT
public:

    constexpr const static qint32 maxNumberOfMeasures = 100;

    constexpr const static qint32 measureDim = 4;

    explicit Camera(const QString& pattern, bool init = true, qint32 port = -1, QObject *parent = 0);

    void initializeCameraInterface();

    void activateObjectiveController(const QString& pattern, qint32 port);

    void setFocusing(const QString& value);

    void setDiaphraghmLevel(const QString& value);

    void setROI(IS_SIZE_2D size);

    void setTriggerMode(qint32 mode = IS_SET_TRIGGER_OFF);

    void tryToStartCamera();

    void procImageQueue();

    void startRecognition();

    void procImageQueueRec();

    void startLiveVideo();

    void stopLiveVideo();

    void debugRecognizeWithVideo(const QString& path);

    void debugRecognizeWithVideoOneThread(const QString& path, qint32 cameraNum);

    void debugRecognizeProc();

    void unlockAllBuffer();

    void stopStream() {streamIsActive = 0;}

    void setFrameRate(qint32 fps);

    void setPixelClock(qint32 pixelClock);

    void setExposure(double exposure);

    void setAutoExposure(bool ok);

    void setHandlePicture(bool ok) {options.pictureParamFlag = ok;}

    void setSharpness(double sharp) {options.sharp = sharp;}

    void setGain(double gain);

    void setGamma(double gamma) {options.gamma = gamma;}

    void setEqualization(bool ok) {options.equalization = ok;}

    void setWhiteBalance(bool ok) {options.whiteBalance = ok;}

    void setIRCorrection(qint32 value) {options.IRCorrection = value;}

    void setSaturation(qint32 value) {options.saturation = value;}

    void setHue(qint32 value) {options.hue = value;}

    void setRSaturation(qint32 value) {options.rSaturation = value;}

    void setGSaturation(qint32 value) {options.gSaturation = value;}

    void setBSaturation(qint32 value) {options.bSaturation = value;}

    void setShadowCoef(double value) {options.shadowCoef = value;}

    void setShadowThreshold(double value) {options.shadowThreshold = value;}

    void setShadowWindowSize(qint32 value) {options.shadowGaussWindowSize = value;}

    void setRawFrame(qint32 value) {options.rawFrame = value;}

    void setVideoDuration(qint32 value) {options.videoDuration = value;}

    void setRotate(qint32 value) {options.rotate = value;}

    void setDebounceEnable(qint32 value);

    void setDebounceValue(qint32 value);

    QLinkedList <FrameTime>::iterator getLastFrame(bool& ok);

    bool getNextFrame(QLinkedList <FrameTime>::iterator& it);

    bool getReadyVideo(QLinkedList <FrameTime>& video);

    void handleCantTakeVideo();


    void setBallRecognizeFlag(qint32 value)
    {
        streamIsActive = 0;
        QThread::msleep(100);
        if (value)
        {
            sendVideoTimer.setInterval(100);
            sendVideoTimer.start();
            emit messageFromCameraReady("Включаю режим распознавания...");
            startRecognition();
        }
        else
        {
            sendVideoTimer.stop();
            videos.clear();
            emit messageFromCameraReady("Выхожу из режима распознавания.");
            options.ballRecognizeFlag = 0;
        }
    }


    void setBallRecognizeFlagDebug(qint32 value)
    {
        options.debugRecFlag = value;
        recognizer.setDebug(value);
    }

    void setBallRecognizeStep(qint32 value) {options.ballRecognizeStep = value;}

    void setWBROI(Rect r) {options.wbRect = r;}

    qint32 getFrameRate() {return options.frameRate;}

    qint32 getVideoDuration() {return options.videoDuration;}
    
    void assignCameraOptions(const CameraOptions& options);

    void fillCurrentCameraParameters();

    const CurrentCameraParams& getCurrentCameraParameters() {return currentParameters;}

    void handlePicture(Mat &rawframe, Mat& frame);

    qint32 getHeight() const {return options.AOIHeight;}

    qint32 getWidth() const {return options.AOIWidth;}

    qint32 getFps() const {return options.frameRate;}

    qint32 getRotate() const {return options.rotate;}

    QString getFocusing();

    ObjectiveController& getObjectiveControllerHandler()  {return controller;}

    void saveSettings();

    ~Camera();

signals:

    void frameReady(cv::Mat frame, QTime t, quint64 internalT);

    void messageFromCameraReady(const QString& str);

    void parametersChanged();

    void currentTimeReady(quint64 t);

    void ballCoordinatesReady(double array[maxNumberOfMeasures][measureDim], qint32 frameCount, qint32 size);

    void hitCoordinatesReady(double array[maxNumberOfMeasures][measureDim], qint32 size);

    void videoReadyToSend();

private:

    void recognitionProcInternal();

    void loadSettings();

    bool camSeqBuild();

    bool camSeqKill();

    void doAutoExposure();

    void debugRecognizeWithVideoInternal(const QString& path);

    ObjectiveController controller;
    qint32	m_nNumberOfBuffers = -1;
    QVector <qint32> m_viSeqMemId;		// camera memory - buffer IDs
    QVector <char*> m_vpcSeqImgMem;	// camera memory - pointers to buffer
    QLinkedList <FrameTime> bufferFrames;
    QAtomicInt streamIsActive;
    qint32 maxBufferSize = 1000;
    quint32 hCam = 0;
    qint32 format = IS_CM_SENSOR_RAW8;//;/*IS_CM_RGB8_PACKED;*/
    qint32 bitCount = 8;
    CameraOptions options;
    QMutex optionMutex;
    QMutex recMutex;
    AutoExposureHandler autoExpHandler;
    CurrentCameraParams currentParameters;
    BallRecognizer recognizer;
    QMetaObject::Connection conn;
    QString str;
    qint32 cameraNumN;
    QLinkedList <VideoToSend> videos;    
    QTimer sendVideoTimer;
    bool videoTaken = false;


};

#endif // CAMERA_H
