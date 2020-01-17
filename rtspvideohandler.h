#ifndef RTSPVIDEOHANDLER_H
#define RTSPVIDEOHANDLER_H

#include <gst/gst.h>
#include <glib.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <QObject>
#include <camera.h>
#include <cameraclient.h>
#include <QApplication>

struct RtspVideoHandlerParams
{
    Camera* cam;
    qint32 w;
    qint32 h;
    qint32 framerate;
    qint32 rotate;
};

class CameraClient;



class RtspVideoHandler : public QObject
{
    Q_OBJECT
public:
    explicit RtspVideoHandler(const RtspVideoHandlerParams& p, CameraClient* clnt, QObject *parent = 0);
    explicit RtspVideoHandler(QObject *parent = 0);

    explicit RtspVideoHandler(const RtspVideoHandlerParams& p, const QLinkedList <FrameTime>& resFrames, QObject *parent = 0);

    void setCameraClient(CameraClient* c) {client = c;}

    void setRtspParams(const RtspVideoHandlerParams& p){params = p;}

    void setVideoToSend(const QLinkedList <FrameTime>& frames) {framesToSend = frames;}

    void sendVideo(qint32 port, const QString &pipeLine, qint32 frameCount = -1);

    bool isSendArray() {return !framesToSend.isEmpty();}

    bool noClientConnected() {return server != nullptr && rtspClient == nullptr;}

    void closeServer();

    ~ RtspVideoHandler();
signals:
    void serverStarted();
public slots:

private:

    void needData (GstElement * appsrc, guint unused, gpointer user_data);

    void needDataArray (GstElement * appsrc, guint unused, gpointer user_data);

    void mediaConfigure (GstRTSPMediaFactory * factory, GstRTSPMedia * media,
                         gpointer user_data);
    void handleClient(GstRTSPServer * self,
                      GstRTSPClient * object,
                      gpointer data);

    void closedClient (GstRTSPClient * self,
                       gpointer data);


    static void needDataCallBack(GstElement* appsrc, guint unused, gpointer data) {
        auto sender = reinterpret_cast <RtspVideoHandler*> (data);
        if (!sender->isSendArray())
        {
            sender->needData(appsrc, unused, data);
        }
        else
        {
            sender->needDataArray(appsrc, unused, data);
        }

    }
    static void mediaConfigureCallBack(GstRTSPMediaFactory * factory, GstRTSPMedia * media, gpointer data) {
        reinterpret_cast<RtspVideoHandler*>(data)->mediaConfigure(factory, media, data);
    }

    static void handleClientCallBack(GstRTSPServer * self,
                                     GstRTSPClient * object,
                                     gpointer data)
    {
        reinterpret_cast<RtspVideoHandler*>(data)->handleClient(self, object, data);
    }

    static void closedClientCallback(GstRTSPClient * self,
                                     gpointer data)
    {
        reinterpret_cast<RtspVideoHandler*>(data)->closedClient(self, data);
    }

    GMainLoop* loop = nullptr;
    GstRTSPServer* server = nullptr;
    RtspVideoHandlerParams params;
    QLinkedList <FrameTime> framesToSend;
    CameraClient* client = nullptr;
    QLinkedList <FrameTime>::iterator it;

    GstRTSPClient* rtspClient = nullptr;
    bool getValidIterator = false;

    qint32 currentFrameCount = 0;
    qint32 needFrameCount = -1;
    qint32 serverId;
    qint64 startTime = -1;

    void reset();
};

#endif // RTSPVIDEOHANDLER_H
