#ifndef CAMERACLIENT_H
#define CAMERACLIENT_H

#include <rtspvideohandler.h>
#include <QObject>
#include <QTcpSocket>
#include <QHostAddress>
#include <QtGlobal>
#include <opencv2/core.hpp>
#include <camera.h>
#include <QScopedPointer>
#include <QSharedPointer>
#include <QtGlobal>
#include <QtConcurrent/QtConcurrent>
#include <QSettings>
#include <QAtomicInteger>
#include <networkmanager.h>

Q_DECLARE_METATYPE(cv::Mat)
Q_DECLARE_METATYPE(FrameInfo)

class RtspVideoHandler;
class CameraClient : public QObject
{
    Q_OBJECT
public:

    explicit CameraClient(Camera* cam, const QString& ipPort,  QObject *parent = 0);



    void sendTest(const QString& test);

    void sendRecognizeResults(const msg::RecognizeData& measures);

    void sendBallCoordinates(const msg::BallMeasure& singleMeasure);

    void sendBallOutOfFrame(const msg::OutOfFrame msg, const Point2f& point);

    qint32 isCollectData() {return collectData;}

    ~CameraClient();

signals:

    void readyMessageFromClient(const QString& msg);

    void internalMessageReady(const QString& msg);

    void sendFinished();

private:

    void loadSettings();

    void saveSettings();

    void initializeMessageHandlers();

    void sendCurrentTimeInternal(quint64 t, QTime ct);

    QScopedPointer <RtspVideoHandler> mainSender;
    QScopedPointer <RtspVideoHandler> addSender;
    Camera* camera;
    bool rawTCP = false;
    QHostAddress sAddress;
    qint32 mainPort;
    QAtomicInteger <quint8> collectData = 0;
    QTimer connectTimer;
    NetworkManager client;


};


#endif // CAMERACLIENT_H
