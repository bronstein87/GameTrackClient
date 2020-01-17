#include "rtspvideohandler.h"

RtspVideoHandler::RtspVideoHandler(const RtspVideoHandlerParams &p, CameraClient* clnt, QObject *parent) : params(p), client(clnt), QObject(parent)
{

}

RtspVideoHandler::RtspVideoHandler(QObject *parent) : QObject(parent)
{

}


RtspVideoHandler::RtspVideoHandler(const RtspVideoHandlerParams& p, const QLinkedList <FrameTime>& resFrames, QObject* parent)
    : params(p), framesToSend(resFrames), QObject(parent)
{

}


void RtspVideoHandler::needData (GstElement* appsrc, guint unused, gpointer user_data)
{
    Q_UNUSED(unused);
    Q_UNUSED(user_data);
    GstBuffer* buffer;
    guint size;
    GstFlowReturn ret;

    bool sendFirst = false;
    if (!getValidIterator)
    {
        it = params.cam->getLastFrame(getValidIterator);
        sendFirst = true;
        if (!getValidIterator)
            return;
    }
    if (params.cam->getNextFrame(it) || sendFirst)
    {
        size = params.w * params.h * 3;
        Mat frame;
        QElapsedTimer t;
        t.start();
        frame = (*it).frame;
        if (params.rotate)
        {
            cvtColor((*it).frame, frame, CV_BayerBG2RGB);
        }
        else
        {
            cvtColor((*it).frame, frame, CV_BayerBG2BGR);
        }

        buffer = gst_buffer_new_wrapped(frame.data, size);

        if (startTime == -1)
        {
            startTime = it->time;
        }
        qint64 ts = (it->time - startTime) * 1e2;
        GST_BUFFER_PTS (buffer) = ts;
        GST_BUFFER_DTS (buffer) = ts;

        qint64 duration = ((double)1 / params.framerate) * GST_SECOND;
        GST_BUFFER_DURATION(buffer) = duration;
        GST_BUFFER_OFFSET(buffer) = currentFrameCount;
        //qDebug() << t.elapsed() << currentFrameCount;
        g_signal_emit_by_name (appsrc, "push-buffer", buffer, &ret);
        frame.addref();
        gst_buffer_unref (buffer);
        ++currentFrameCount;
    }

}

void RtspVideoHandler::needDataArray(GstElement *appsrc, guint unused, gpointer user_data)
{
    Q_UNUSED(unused);
    Q_UNUSED(user_data);
    GstBuffer *buffer;
    guint size;
    GstFlowReturn ret;

    if (!getValidIterator)
    {
        it = framesToSend.begin();
        getValidIterator = true;
    }
    else
    {
        ++it;
    }

    if (it == framesToSend.end())
    {
        it = --framesToSend.end();
    }


    size = params.w * params.h * 3;
    Mat frame;
    if (params.rotate)
    {
        cvtColor((*it).frame, frame, CV_BayerBG2RGB);
    }
    else
    {
        cvtColor((*it).frame, frame, CV_BayerBG2BGR);
    }
    buffer = gst_buffer_new_wrapped(frame.data, size);
    if (startTime == -1)
    {
        startTime = it->time;
    }
    qint64 ts = (it->time - startTime) * 1e2;
    GST_BUFFER_PTS (buffer) = ts;
    GST_BUFFER_DTS (buffer) = ts;

    qint64 duration = ((double)1 / params.framerate) * GST_SECOND;
    GST_BUFFER_DURATION(buffer) = duration;
    GST_BUFFER_OFFSET(buffer) = currentFrameCount;
    g_signal_emit_by_name (appsrc, "push-buffer", buffer, &ret);
    frame.addref();
    gst_buffer_unref (buffer);
    ++currentFrameCount;

}

/* called when a new media pipeline is constructed. We can query the
 * pipeline and configure our appsrc */
void RtspVideoHandler::mediaConfigure (GstRTSPMediaFactory* factory, GstRTSPMedia* media,
                                       gpointer user_data)
{
    Q_UNUSED(factory);
    Q_UNUSED(user_data);
    GstElement *element, *appsrc;
    /* get the element used for providing the streams of the media */
    element = gst_rtsp_media_get_element (media);

    /* get our appsrc, we named it 'mysrc' with the name property */
    appsrc = gst_bin_get_by_name_recurse_up (GST_BIN (element), "vsrc");

    /* this instructs appsrc that we will be dealing with timed buffer */
    gst_util_set_object_arg (G_OBJECT (appsrc), "format", "time");
    /* configure the caps of the video */
    //    g_object_set (G_OBJECT (appsrc), "caps",
    //                  gst_caps_new_simple ("video/x-raw(memory:NVMM)",
    //                                       "format", G_TYPE_STRING, "BGR",
    //                                       "width", G_TYPE_INT, params.w,
    //                                       "height", G_TYPE_INT, params.h,
    //                                       "framerate", GST_TYPE_FRACTION, params.framerate, 1, NULL)
    //                  , NULL);

    //g_object_set (appsrc, "do-timestamp", true, NULL);
    // g_object_set(appsrc, "is-live", true, NULL);
    //g_object_set(appsrc, "min-latency", 0, NULL);
    g_object_set (G_OBJECT (appsrc), "caps",
                  gst_caps_new_simple ("video/x-raw",
                                       "format", G_TYPE_STRING, "BGR",
                                       "width", G_TYPE_INT, params.w,
                                       "height", G_TYPE_INT, params.h,
                                       "framerate", GST_TYPE_FRACTION, params.framerate, 1, NULL)
                  , NULL);

    /* install the callback that will be called when a buffer is needed */
    g_signal_connect (appsrc, "need-data", (GCallback) needDataCallBack, this);
    gst_object_unref (appsrc);
    gst_object_unref (element);
}

void RtspVideoHandler::handleClient(GstRTSPServer *self, GstRTSPClient *object, gpointer data)
{
    qDebug() << "NEW CLIENT";
    Q_UNUSED(self);
    Q_UNUSED(data);
    rtspClient = object;
    g_signal_connect (rtspClient, "closed", (GCallback) closedClientCallback, this);
}

void RtspVideoHandler::closedClient(GstRTSPClient *self, gpointer data)
{
    qDebug() << "client CLOSED";
    Q_UNUSED(self);
    Q_UNUSED(data);
    rtspClient = nullptr;
    closeServer();
}



void RtspVideoHandler::reset()
{
    getValidIterator = false;
    currentFrameCount = 0;
    it = QLinkedList <FrameTime>::iterator();
    startTime = -1;
}

void RtspVideoHandler::sendVideo(qint32 port, const QString& pipeLine, qint32 frameCount)
{
    reset();
    if (frameCount != -1)
    {
        needFrameCount = frameCount;
    }
    setenv("GST_DEBUG","4", 0);
    loop = g_main_loop_new (NULL, FALSE);

    server = gst_rtsp_server_new();
    g_signal_connect (server, "client-connected", (GCallback) handleClientCallBack, this);
    g_object_set (server, "service", QString::number(port).toStdString().c_str(), NULL);

    GstRTSPMountPoints* mounts = gst_rtsp_server_get_mount_points (server);
    GstRTSPMediaFactory* factory = gst_rtsp_media_factory_new ();

    gst_rtsp_media_factory_set_launch (factory,
                                       QString("( %1 )").arg(pipeLine).toStdString().c_str());

    g_signal_connect (factory, "media-configure", (GCallback) mediaConfigureCallBack, this);

    gst_rtsp_mount_points_add_factory (mounts, "/vd", factory);

    g_object_unref (mounts);

//    GstRTSPThreadPool* tp = gst_rtsp_server_get_thread_pool (server);
//    qDebug() << gst_rtsp_thread_pool_get_max_threads(tp) << "MAX THREADS";
//    gst_rtsp_thread_pool_set_max_threads(tp, 0);
    serverId = gst_rtsp_server_attach (server, NULL);

    emit serverStarted();
    QApplication::processEvents();
    qDebug() << "server STARTED";
    g_main_loop_run (loop);
}

void RtspVideoHandler::closeServer()
{
    if (server != nullptr)
    {
        reset();
        if (rtspClient != nullptr)
        {
            gst_rtsp_client_close(rtspClient);
            rtspClient = nullptr;
        }
        g_source_remove (serverId);
        GstRTSPMountPoints *mounts = nullptr;
        mounts = gst_rtsp_server_get_mount_points(server);
        gst_rtsp_mount_points_remove_factory (mounts, "/vd");
        g_object_unref (mounts);
        g_object_unref(server);
        server = nullptr;
        qDebug() << loop;
        if (loop != nullptr)
        {
            qDebug() <<"stop";
            g_main_loop_quit(loop);
            loop = nullptr;
        }

    }
}

RtspVideoHandler::~RtspVideoHandler()
{
    closeServer();
}

