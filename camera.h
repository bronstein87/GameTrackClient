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
#include <ballrecognizerpp.h>
#include <QtConcurrent/QtConcurrent>
#include <mainstructs.h>
#include <mathfunc.h>
#include <proto/msg.internal.pb.h>
#include <proto/proto_helper.h>
#include <logger.h>
#include <battracker.h>
#include <pedestriantracker.h>
#include <QJsonDocument>

//#define AUTOEXP_MAIN_THREAD

using namespace BOKZMath;
using namespace gt::internal;
using namespace imageprocext;




class Camera : public QObject
{
    Q_OBJECT
public:

    explicit Camera(const QString& pattern, bool init = true, qint32 port = -1, QObject *parent = 0);

    void initializeCameraInterface();

    void activateObjectiveController(const QString& pattern, qint32 port);

    void setFocusing(const QString& value);

    void setDiaphraghmLevel(const QString& value);

    void setROI(IS_SIZE_2D size);

    void setTriggerModeEnable(bool enable);

    void setTriggerMode(qint32 mode = IS_SET_TRIGGER_OFF);

    void tryToStartCamera();

    void procImageQueue();

    //void procImageImitation();

    void startRecognition();

    void resetWaitForCommand() {waitForCommand = false;}

    void setFramePosition(qint32 offset) {moveFrames = offset;}

    void startLiveVideo();

    void stopLiveVideo();

    void unlockAllBuffer();

    void stopStream() {streamIsActive = 0;}

    void setFrameRate(qint32 fps);

    void setPixelClock(qint32 pixelClock);

    void setExposure(double exposure);

    void setAutoExposure(bool ok);

    void setHandlePicture(bool ok) {options.mutable_p_params()->set_picture_param_flag(ok);}

    void setSharpness(double sharp) {options.mutable_p_params()->set_sharp(sharp);}

    void setGain(double gain);

    void setGamma(double gamma) {options.mutable_p_params()->set_gamma(gamma);}

    void setWhiteBalance(bool ok) {options.mutable_p_params()->set_white_balance(ok);}

    void setSaturation(qint32 value) {options.mutable_p_params()->set_saturation(value);}

    void setHue(qint32 value) {options.mutable_p_params()->set_hue(value);}

    void setRSaturation(qint32 value) {options.mutable_p_params()->set_r_saturation(value);}

    void setGSaturation(qint32 value) {options.mutable_p_params()->set_g_saturation(value);}

    void setBSaturation(qint32 value) {options.mutable_p_params()->set_b_saturation(value);}

    void setShadowCoef(double value) {options.mutable_p_params()->set_shadow_coef(value);}

    void setShadowThreshold(double value) {options.mutable_p_params()->set_shadow_threshold(value);}

    void setShadowWindowSize(qint32 value) {options.mutable_p_params()->set_shadow_gauss_window_size(value);}

    void setRawFrame(qint32 value) {options.mutable_stream_params()->set_raw_frame(value);}

    void setRotate(qint32 value) {options.mutable_p_params()->set_rotate(value);}

    void setWBROI(cv::Rect r) {options.mutable_p_params()->set_allocated_wb_rect(ProtoHelper::cvRectToGt(r));}

    void setDebounceEnable(bool enable);

    void setDebounceValue(qint32 value);

    void setHWParams(const msg::CameraOptions& _options, bool initialize);

    void setBallRecognizeFlag(qint32 value);

    void setBallRecognizeFlagDebug(bool value);

    QLinkedList <FrameInfo>::iterator getLastFrame(bool& ok, bool main);

    void updateFrameState(QLinkedList<FrameInfo>::iterator& it, bool handled, bool sent);

    bool getNextFrame(QLinkedList <FrameInfo>::iterator& it, bool main);

    void assignCameraOptions(const msg::CameraOptions& _options);

    void fillCurrentCameraParameters();

    QString getFocusing();

    ObjectiveController& getObjectiveControllerHandler()  {return controller;}

    const msg::CameraOptions& getOptions() {return options;}

    void saveSettings();

    //void debugPedestrianDraw(const QString& path);

    ~Camera();



signals:

    void frameReady(const FrameInfo& ft);

    void messageFromCameraReady(const QString& str);

    void parametersChanged(msg::CameraOptions options);

    void currentTimeReady(quint64 t, QTime ct);

    void ballCoordinatesReady(const msg::RecognizeData& measures);

    void hitCoordinatesReady(const msg::RecognizeData& measures);

    void ballMeasureReady(const msg::BallMeasure& measure);

    void ballOutOfFrame(const  msg::OutOfFrame& msg, const Point2f& point);

private:

    void recognitionProcInternal();

    void loadSettings();

    bool camSeqBuild();

    bool camSeqKill();

    void doAutoExposure();

    void debugRecognizeWithVideoInternal(const QString& path);

    void getHWExposure(qint32 errorCode);

    void getHWExposureRange(qint32 errorCode);

    void getHWPixelClock(qint32 errorCode);

    void getHWPixelClockRange(qint32 errorCode);

    void getHWFrameTimeRange(qint32 errorCode);

    void getHWFrameTime(qint32 errorCode);

    void getHWTriggerDebounce(qint32 errorCode);

    void getHWDebounceValue(qint32 errorCode);

    void updateHWParameters();

    ObjectiveController controller;
    QVector <qint32> m_viSeqMemId;  // camera memory - buffer IDs
    QVector <char*> m_vpcSeqImgMem;	// camera memory - pointers to buffer
    QLinkedList <FrameInfo> bufferFrames;
    QAtomicInt streamIsActive;
    constexpr const static qint32 maxBufferSize = 600;
    quint32 hCam = 0;
    qint32 format = IS_CM_SENSOR_RAW8;//;/*IS_CM_RGB8_PACKED;*/
    qint32 bitCount = 8;
    msg::CameraOptions options;
    QMutex optionMutex;
    QMutex recMutex;
    AutoExposureHandler autoExpHandler;
    BallRecognizerPP recognizer;
    //PedestrianTracker pedTracker;
    //BatTracker batTracker; // tmp
    QTimer autoExpTimer;
    constexpr static const qint32 delayTreshold = 1000;

    QString debugVideoPath;
    QString debugTimePath;
    qint32 moveFrames = 0;
    bool waitForCommand = false;
};

#endif // CAMERA_H
