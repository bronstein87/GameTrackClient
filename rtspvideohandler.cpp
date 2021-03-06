#include "rtspvideohandler.h"




GstPadProbeReturn
RtspVideoHandler::extend_rtp_header_probe (GstPad* pad,
                                           GstPadProbeInfo* info,
                                           gpointer         user_data)
{
    Q_UNUSED(pad);
    RtspVideoHandler* handler = (RtspVideoHandler*)user_data;
    GstBuffer *buffer;
    GstRTPBuffer rtpBuffer = GST_RTP_BUFFER_INIT;

    buffer = GST_PAD_PROBE_INFO_BUFFER (info);
    buffer = gst_buffer_make_writable (buffer);

    if (buffer == NULL)
    {
        return GST_PAD_PROBE_OK;
    }
    if (gst_rtp_buffer_map (buffer,GST_MAP_WRITE, &rtpBuffer))
    {
        static guint32 tstmp = -1;
        guint32 ts = gst_rtp_buffer_get_timestamp(&rtpBuffer);
        if (tstmp != ts)
        {
            QMutexLocker lock(&handler->mutex);
            Q_ASSERT(!handler->tsRtpHeader.isEmpty());
            quint64 data = handler->tsRtpHeader.first();
            handler->tsRtpHeader.removeFirst();
            lock.unlock();
            if (gst_rtp_buffer_add_extension_onebyte_header(&rtpBuffer, 1, &data, sizeof(data)) != TRUE)
            {
                g_printerr("cannot add extension to rtp header");
            }
        }
        tstmp = ts;
        gst_rtp_buffer_unmap (&rtpBuffer);
    }

    return GST_PAD_PROBE_OK;
}

RtspVideoHandler::RtspVideoHandler(const RtspVideoHandlerParams &p, QObject *parent) : params(p),  QObject(parent)
{
    init();
}

RtspVideoHandler::RtspVideoHandler(QObject *parent) : QObject(parent)
{
    init();
}


RtspVideoHandler::RtspVideoHandler(const RtspVideoHandlerParams& p, const QLinkedList <FrameInfo>& resFrames, QObject* parent)
    : params(p), framesToSend(resFrames), QObject(parent)
{
    init();
}


void RtspVideoHandler::needData (GstElement* appsrc, guint unused, gpointer user_data)
{
    Q_UNUSED(unused);
    Q_UNUSED(user_data);
    GstBuffer* buffer;
    guint size;
    GstFlowReturn ret;
    QElapsedTimer t;
    t.start();
    if (!getValidIterator)
    {
        it = params.cam->getLastFrame(getValidIterator, onlyMain);
        if (!getValidIterator)
        {
            return;
        }
    }


    if (!it->sent || params.cam->getNextFrame(it, onlyMain))
    {
        size = params.w * params.h * 4;
        Mat frame;
        if (params.isDebugMode)
        {
            cvtColor(it->frame, frame, COLOR_BGR2RGBA);
        }
        else
        {
            if (params.isRotated)
            {
                cvtColor(it->frame, frame, COLOR_BayerRG2RGBA);
            }
            else
            {
                cvtColor(it->frame, frame, COLOR_BayerBG2RGBA);
            }
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
        QMutexLocker lock(&mutex);
        if (needFrameCount != -1 && currentFrameCount >= needFrameCount)
        {
            tsRtpHeader.append(0);
        }
        else
        {
            tsRtpHeader.append(it->time);
        }
        lock.unlock();

        g_signal_emit_by_name (appsrc, "push-buffer", buffer, &ret);
        params.cam->updateFrameState(it, it->handled, true);

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
        it = framesToSend.erase(it);
    }

    if (it == framesToSend.end())
    {
        g_signal_emit_by_name (appsrc, "push-buffer", gst_buffer_new(), &ret);
        //it = --framesToSend.end();
    }

    size = params.w * params.h * 3;
    Mat frame;
    frame = (*it).frame;
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
    GstElement *element, *appsrc, *rtph264pay;
    GstPad *pad;
    /* get the element used for providing the streams of the media */
    element = gst_rtsp_media_get_element (media);

    appsrc = gst_bin_get_by_name_recurse_up (GST_BIN (element), "vsrc");
    rtph264pay = gst_bin_get_by_name_recurse_up (GST_BIN (element), "pay0");
    pad = gst_element_get_static_pad (rtph264pay, "src");
    gst_pad_add_probe (pad, GST_PAD_PROBE_TYPE_BUFFER,
                       (GstPadProbeCallback) extend_rtp_header_probe, this, NULL);
    gst_object_unref (pad);

    /* this instructs appsrc that we will be dealing with timed buffer */
    gst_util_set_object_arg (G_OBJECT (appsrc), "format", "time");
    /* configure the caps of the video */
    g_object_set (G_OBJECT (appsrc), "caps",
                  gst_caps_new_simple ("video/x-raw",
                                       "format", G_TYPE_STRING, "RGBA",
                                       "width", G_TYPE_INT, params.w,
                                       "height", G_TYPE_INT, params.h,
                                       "framerate", GST_TYPE_FRACTION, params.framerate, 1, NULL)
                  , NULL);

    /* install the callback that will be called when a buffer is needed */
    g_signal_connect (appsrc, "need-data", (GCallback) needDataCallBack, this);
    gst_object_unref (appsrc);
    gst_object_unref (element);
    gst_object_unref(rtph264pay);
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

void RtspVideoHandler::init()
{
    timeoutTimer.setInterval(5000);
    connect(this, &RtspVideoHandler::serverStarted, this, [this]()
    {
        timeoutTimer.start();
    });

    connect(&timeoutTimer, &QTimer::timeout, this, [this]()
    {
        // qDebug() << "try timeout";
        if (noClientConnected())
        {
            qDebug() << "timeout";
            closeServer();
        }
        timeoutTimer.stop();
    });
}



void RtspVideoHandler::reset()
{
    framesToSend.clear();
    getValidIterator = false;
    currentFrameCount = 0;
    it = QLinkedList <FrameInfo>::iterator();
    startTime = -1;
}

void RtspVideoHandler::sendVideo(qint32 port, const QString& pipeLine, qint32 frameCount, bool _onlyMain)
{
    reset();
    if (frameCount != -1)
    {
        needFrameCount = frameCount;
    }
    onlyMain = _onlyMain;
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

    serverId = gst_rtsp_server_attach (server, NULL);

    emit serverStarted();
    QApplication::processEvents();
    qDebug() << "server STARTED";
    g_main_loop_run (loop);
}

void RtspVideoHandler::setTimeout(qint32 ms)
{
    timeoutTimer.setInterval(ms);
}

void RtspVideoHandler::closeServer()
{
    if (server != nullptr)
    {
        qDebug() << "rtsp destructor";
        reset();
        if (rtspClient != nullptr)
        {
            gst_rtsp_client_close(rtspClient);
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
            qDebug() << "stop";
            g_main_loop_quit(loop);
            loop = nullptr;
        }
    }
}

void RtspVideoHandler::disconnectClient()
{
    if (rtspClient != nullptr)
    {
        gst_rtsp_client_close(rtspClient);
    }
}

RtspVideoHandler::~RtspVideoHandler()
{
    closeServer();
}

