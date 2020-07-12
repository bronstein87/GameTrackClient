#ifndef RTSPVIDEOHANDLER_H
#define RTSPVIDEOHANDLER_H

#include <gst/gst.h>
#include <glib.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/rtp/gstrtpbuffer.h>
#include <QObject>
#include <camera.h>
#include <QApplication>


struct RtspVideoHandlerParams
{
    Camera* cam;
    qint32 w;
    qint32 h;
    qint32 framerate;
};

class CameraClient;



class RtspVideoHandler : public QObject
{
    Q_OBJECT
public:
    explicit RtspVideoHandler(const RtspVideoHandlerParams& p, QObject *parent = 0);

    explicit RtspVideoHandler(QObject *parent = 0);

    explicit RtspVideoHandler(const RtspVideoHandlerParams& p, const QLinkedList <FrameInfo>& resFrames, QObject *parent = 0);

    void setRtspParams(const RtspVideoHandlerParams& p){params = p;}

    void setVideoToSend(const QLinkedList <FrameInfo>& frames) {framesToSend = frames;}

    void sendVideo(qint32 port, const QString &pipeLine, qint32 frameCount = -1, bool _onlyMain = true);

    void setTimeout(qint32 ms);

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

    void init();

    void reset();


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

    static GstPadProbeReturn
    extend_rtp_header_probe (GstPad* pad,
                GstPadProbeInfo* info,
                gpointer         user_data);

    CameraClient* client = nullptr;
    RtspVideoHandlerParams params;
    QLinkedList <FrameInfo> framesToSend;
    QLinkedList <FrameInfo>::iterator it;
    QLinkedList <quint64> tsRtpHeader;
    QMutex mutex;



    GMainLoop* loop = nullptr;
    GstRTSPServer* server = nullptr;
    GstRTSPClient* rtspClient = nullptr;


    qint32 currentFrameCount = 0;
    qint32 needFrameCount = -1;
    qint32 serverId;
    qint64 startTime = -1;
    bool onlyMain = false;
    bool getValidIterator = false;
    QTimer timeoutTimer;


};

#endif // RTSPVIDEOHANDLER_H
