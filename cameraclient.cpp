#include "cameraclient.h"

using namespace cv;
CameraClient::CameraClient(Camera* cam, const QString& ipPort, QObject *parent) : client(NetworkManager::EndPointType::Client, ipPort, false), camera(cam),
    mainSender(new RtspVideoHandler),
    addSender(new RtspVideoHandler),
    QObject(parent)
{
    //camera->batTracker.initCNN("/home/nvidia/Downloads/net/deploy.prototxt", "/home/nvidia/Downloads/net/snapshot_iter_340400.caffemodel", 0.6);
    qRegisterMetaType <cv::Mat> ();
    qRegisterMetaType <FrameInfo>();
    qRegisterMetaType <msg::CameraOptions>();

    connect(camera, &Camera::messageFromCameraReady, this, [this](auto msg){this->sendTest(msg);});
    connect(this, &CameraClient::internalMessageReady, this, [this](auto msg){this->sendTest(msg);});
    connect(camera, &Camera::currentTimeReady, this, &CameraClient::sendCurrentTimeInternal);
    connect(camera, &Camera::ballCoordinatesReady, this, &CameraClient::sendRecognizeResults);
    connect(camera, &Camera::hitCoordinatesReady, this, &CameraClient::sendRecognizeResults);
    connect(camera, &Camera::ballMeasureReady, this, &CameraClient::sendBallCoordinates);
    connect(camera, &Camera::ballOutOfFrame, this, &CameraClient::sendBallOutOfFrame);
    connect(camera, &Camera::parametersChanged, this, [this](auto parameters)
    {
        client.send(msg::GameTrackProtocol::SendCameraParameters, &parameters);
    });

    initializeMessageHandlers();
    client.initialize();

}



void CameraClient::sendTest(const QString& test)
{
    msg::DebugInfo info;
    info.set_message(test.toStdString());
    client.send(msg::GameTrackProtocol::GetTestData, &info);
}


void CameraClient::sendRecognizeResults(const msg::RecognizeData& measures)
{
    client.send(msg::GameTrackProtocol::GetBallCoordinates, &measures);
}

void CameraClient::sendBallCoordinates(const msg::BallMeasure& singleMeasure)
{
    client.send(msg::GameTrackProtocol::NotifyBallMeasure, &singleMeasure);
}

void CameraClient::sendBallOutOfFrame(const msg::OutOfFrame msg, const Point2f &point)
{
    msg::OutOfFrameInfo info;
    info.set_dir(msg);
    info.mutable_ball_measure()->mutable_xy()->set_x(point.x);
    info.mutable_ball_measure()->mutable_xy()->set_x(point.y);
    client.send(msg::GameTrackProtocol::NotifyOutOfFrame, &info);
}


void CameraClient::sendCurrentTimeInternal(quint64 t, QTime ct)
{
    msg::FrameTime ft;
    ft.set_camera_time(t);
    ft.set_computer_time(ct.msecsSinceStartOfDay());
    client.send(msg::GameTrackProtocol::SendCurrentTime, &ft);
}

void CameraClient::initializeMessageHandlers()
{

    client.setHandler(NetworkManager::OnConnect, [this](const NetworkManager::MessageData& data)
    {
        Q_UNUSED(data);
        connectTimer.stop();
        camera->fillCurrentCameraParameters();
        client.send(msg::GameTrackProtocol::SendCameraParameters, &camera->getOptions());
    });

    client.setHandler(NetworkManager::OnDisconnect, [this](const NetworkManager::MessageData& data)
    {
        connect(&connectTimer, &QTimer::timeout, [this, data]()
        {
            qDebug() << data.msg.size() << data.msg <<  data.msg.toUInt();
            quint32 port;
            memcpy(&port, data.msg, data.msg.size());
            data.socket->connectToHost(data.socket->peerName(), port);
        });
        connectTimer.setInterval(1000);
        connectTimer.start();
    });


    client.setHandler(msg::GameTrackProtocol::SendCameraParameters, [this](const NetworkManager::MessageData& data)
    {
        msg::CameraOptions opt;
        opt.ParseFromArray(data.msg.data(), data.msg.size());
        camera->assignCameraOptions(opt);
    });

    client.setHandler(msg::GameTrackProtocol::RequestStream, [this](const NetworkManager::MessageData& data)
    {
        camera->tryToStartCamera();
        msg::StreamCameraCommand command;
        command.ParseFromArray(data.msg.data(), data.msg.size());
        if (command.has_type())
        {
            if (command.has_stop() && command.stop())
            {
                qDebug() << "stop server";
                if (command.type() == msg::StreamType::Main)
                {
                    mainSender->disconnectClient();
                }
                else
                {

                    addSender->disconnectClient();
                }
            }
            else
            {
                auto options = camera->getOptions();
                RtspVideoHandlerParams params;
                params.cam = camera;
                params.w = options.hw_params().width();
                params.h = options.hw_params().height();
                params.isRotated = options.p_params().rotate();
                params.isDebugMode = options.debug_mode();


                QString pipeLine = QString("( appsrc name=vsrc "
                                           "! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! omxh264enc bitrate=20000000 control-rate=2 "
                                           "! rtph264pay name=pay pt=96 ! identity name=pay0 )")
                        .arg(params.w)
                        .arg(params.h);
                if (command.type() == msg::StreamType::Main)
                {
                    params.framerate = options.stream_params().send_frame_rate_main();
                    pipeLine = pipeLine.arg(params.framerate);
                    mainSender->disconnectClient();
                    mainSender->setRtspParams(params);
                    qDebug() << pipeLine;
                    if (!mainSender->isInitialized())
                    {
                        QtConcurrent::run(mainSender.data(), &RtspVideoHandler::initialize, options.stream_params().port_send_stream_main(), pipeLine, command.video_duration(), true);
                    }
                }
                else
                {
                    params.framerate  = options.stream_params().send_frame_rate_add();
                    pipeLine = pipeLine.arg(params.framerate);
                    addSender->disconnectClient();
                    addSender->setRtspParams(params);
                    //QtConcurrent::run(addSender.data(), &RtspVideoHandler::sendVideo, options.stream_params().port_send_stream_add(), pipeLine, command.video_duration(), false);
                }
                client.send(msg::GameTrackProtocol::RequestStream, &command);
            }

        }
    });

    client.setHandler(msg::GameTrackProtocol::NotifyOutOfFrame, [this](const NetworkManager::MessageData& data)
    {

    });


    client.setHandler(msg::GameTrackProtocol::RequestCameraFrame, [this](const NetworkManager::MessageData& data)
    {
        Q_UNUSED(data);
        camera->stopLiveVideo();
        camera->startLiveVideo();
        camera->tryToStartCamera();
        QSharedPointer <QMetaObject::Connection> conn (new QMetaObject::Connection);
        *conn = connect(camera, &Camera::frameReady, this, [this, conn](FrameInfo ft) mutable
        {
            if (conn)
            {
                QObject::disconnect(*conn);
                conn.clear();
                msg::FrameTime sendFt;
                sendFt.set_camera_time(ft.time);
                sendFt.set_computer_time(ft.computerTime);
                Mat tmp;
                cvtColor(ft.frame, tmp, COLOR_BayerBG2BGR);
                std::string frame((const char*)tmp.data, ft.frame.cols * ft.frame.rows * 3);
                *sendFt.mutable_frame() = frame;
                client.send(msg::GameTrackProtocol::RequestCameraFrame, &sendFt);
            }

        });
    });

    client.setHandler(msg::GameTrackProtocol::RequestNextFrameDebugMode, [this](const NetworkManager::MessageData& data)
    {
        if (!data.msg.isEmpty())
        {
            msg::DebugInfo command;
            command.ParseFromArray(data.msg.data(), data.msg.size());
            if (command.has_message())
            {
                camera->setFramePosition(command.skipdebugframes());
            }
        }
        camera->resetWaitForCommand();
    });

    client.setHandler(msg::GameTrackProtocol::RequestPreviousFrameDebugMode, [this](const NetworkManager::MessageData& data)
    {
        if (!data.msg.isEmpty())
        {
            msg::DebugInfo command;
            command.ParseFromArray(data.msg.data(), data.msg.size());
            camera->setFramePosition(command.skipdebugframes());
        }
        camera->resetWaitForCommand();
    });

}

CameraClient::~CameraClient()
{
    qDebug() << "server destroy";

}

void CameraClient::loadSettings()
{

}

void CameraClient::saveSettings()
{

}


