#ifndef CAMERACLIENT_H
#define CAMERACLIENT_H

#include <rtspvideohandler.h>
#include <QObject>
#include <QTcpSocket>
#include <QHostAddress>
#include <QtGlobal>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudacodec.hpp>
#include <camera.h>
#include <QScopedPointer>
#include <QSharedPointer>
#include <QtGlobal>
#include <QtConcurrent/QtConcurrent>
#include <QSettings>
#include <QAtomicInteger>

Q_DECLARE_METATYPE(cv::Mat)



enum Camera57Protocol
{
    SendCameraParams = 0x50,
    RequestCameraVideo = 0x56,
    RequestCameraFrame = 0x46,
    ReadyToGetStream = 0x52,
    IsServerPrepareToGetStream = 0x51,
    GetThrowCoordinates = 0x44,
    GetHitCoordinates = 0x45,
    GetTestDataFromClient = 0x54,
    StartStream = 0x53,
    StopStream = 0x55,
    RestartCamera = 0x59,
    AskCurrentCameraParams = 0x57,
    SendCurrentFrameTime = 0x60,
    SendSaveParameters = 0x61,
    SendRecognizeVideo = 0x62,
    EndOfMessageSign = 0x23
};



class RtspVideoHandler;
class CameraClient : public QObject
{
    Q_OBJECT
public:

    explicit CameraClient(Camera* cam, QHostAddress address = QHostAddress(), quint16 port = 0,  QObject *parent = 0);

    void connectToCameraServer(const QHostAddress& address, quint16 port);

    void sendTest(const QByteArray& test);

    void sendRecognizeResults(double ballCoordinates[Camera::maxNumberOfMeasures][Camera::measureDim], qint32 frameCount, qint32 size);

    void sendHitResults(double ballCoordinates[Camera::maxNumberOfMeasures][Camera::measureDim], qint32 size);

    qint32 isCollectData() {return collectData;}

    ~CameraClient();


signals:

    void readyMessageFromClient(const QString& msg);

    void internalMessageReady(const QString& msg);

    void sendFinished();


private:

    void loadSettings();

    void saveSettings();

    void handleMessageFromServer();

    void handleError(QAbstractSocket::SocketError socketError);

    void sendVideoInternal(qint32 frameCount);

    void sendFrameInternal();

    void sendCurrentCameraParamsInternal();

    void sendCurrentTimeInternal(quint64 t);

    void sendRawStream(qint32 count);

    void sendBallCoordinates(double array[Camera::maxNumberOfMeasures][Camera::measureDim], qint32 size);

    void fillEndOfMessage(QByteArray& array);

    void sendRecognizeVideo(const QLinkedList <FrameTime>& frames);

    QTcpSocket* client;

    QByteArray data;

    QVector <char> avaliableCommands;

    Camera* camera;

    double frameRateSend = 30.0;
    qint32 streamPort = 5000;
    bool rawTCP = false;
    QHostAddress sAddress;
    qint32 mainPort;
    QAtomicInteger <quint8> collectData = 0;
    QTimer connectTimer;
    QTimer rtspTimeoutTimer;
    QScopedPointer <RtspVideoHandler> sender;


};

class AutoDisconnecter
{
public:
    AutoDisconnecter() {}

    AutoDisconnecter(QMetaObject::Connection conn): connection(conn) {}

    void setConnection(QMetaObject::Connection conn) {connection = conn;}

    ~AutoDisconnecter() {QObject::disconnect(connection);}

private:

    QMetaObject::Connection connection;




};

#endif // CAMERACLIENT_H
