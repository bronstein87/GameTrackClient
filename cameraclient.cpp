#include "cameraclient.h"

using namespace cv;
CameraClient::CameraClient(Camera* cam, QHostAddress address, quint16 port, QObject *parent) : client(new QTcpSocket()),
    avaliableCommands {SendCameraParams, RequestCameraVideo, RequestCameraFrame, ReadyToGetStream,
                       IsServerPrepareToGetStream, GetThrowCoordinates, GetHitCoordinates,
                       GetTestDataFromClient,StartStream,
                       StopStream, RestartCamera, AskCurrentCameraParams, SendCurrentFrameTime,
                       SendSaveParameters, SendRecognizeVideo, EndOfMessageSign}, camera(cam), sender(new RtspVideoHandler), QObject(parent)
{
    qRegisterMetaType <cv::Mat> ();
    loadSettings();
    if (!address.isNull() && port != 0)
    {
        mainPort = port;
        sAddress = address;
        connect(&connectTimer, &QTimer::timeout, [this]() {connectToCameraServer(sAddress, mainPort);});
        connect(client, &QTcpSocket::connected, [this](){connectTimer.stop();});
        connect(client, &QTcpSocket::disconnected, [this]() {connectTimer.start(1000);});
        connectTimer.start(1000);
    }

    connect(client, &QTcpSocket::readyRead, this, &CameraClient::handleMessageFromServer);
    connect(client, static_cast<void (QAbstractSocket::*)(QAbstractSocket::SocketError)>(&QAbstractSocket::error), this, &CameraClient::handleError);
    connect(camera, &Camera::messageFromCameraReady, this, [this](auto msg){this->sendTest(msg.toLocal8Bit());});
    connect(this, &CameraClient::internalMessageReady, this, [this](auto msg){this->sendTest(msg.toLocal8Bit());});
    connect(camera, &Camera::parametersChanged, this, &CameraClient::sendCurrentCameraParamsInternal);
    connect(camera, &Camera::currentTimeReady, this, &CameraClient::sendCurrentTimeInternal);
    connect(camera, &Camera::ballCoordinatesReady, this, &CameraClient::sendRecognizeResults);
    connect(camera, &Camera::hitCoordinatesReady, this, &CameraClient::sendHitResults);
    connect(camera, &Camera::videoReadyToSend, this, [this] ()
    {
        if (collectData == 0)
        {
            QLinkedList <FrameTime> v;
            collectData = 1;
            if (camera->getReadyVideo(v))
            {
                rtspTimeoutTimer.stop();
                QByteArray array;
                array.append(SendRecognizeVideo);
                for (auto& i : v)
                {
                    array.append((char*)&i.time, sizeof(quint64));
                }
                fillEndOfMessage(array);

                QSharedPointer <QMetaObject::Connection> conn (new QMetaObject::Connection);
                *conn = connect(sender.data(), &RtspVideoHandler::serverStarted, this, [array, conn, this]()
                {
                    client->write(array);
                    bool flush = true;
                    while (flush)
                    {
                        flush = client->flush();
                    }
                    //qDebug() << "SEND SIGNAL";
                    rtspTimeoutTimer.start(3000);
                    disconnect(*conn);
                });

                QtConcurrent::run(this, &CameraClient::sendRecognizeVideo, v);
            }
            else
            {
                collectData = 0;
            }
        }
        else
        {
            camera->handleCantTakeVideo();
        }

    });
    connect(&rtspTimeoutTimer, &QTimer::timeout, this, [this]()
    {
       // qDebug() << "try timeout";
        if (sender->noClientConnected())
        {
            qDebug() << "timeout";
            sender->closeServer();
        }
        rtspTimeoutTimer.stop();
    });
    //    sender.sendVideo(4511, QString("appsrc name=vsrc ! videoconvert ! video/x-raw,format=I420,width=1920,height=1080,framerate=60/1 "
    //                                   "! omxh265enc bitrate=20000000 control-rate=2 "
    //                                   "! rtph265pay name=pay0 pt=96"));
    //    connect(sender.data(), &RtspVideoHandler::serverStarted, this, [this]()
    //                    {
    //                       qDebug() << sender->noClientConnected();
    //                       sender->closeServer();
    //                       qDebug() << "qq";
    //                    });
    //   QtConcurrent::run(sender.data(), &RtspVideoHandler::sendVideo, 4511, QString("appsrc name=vsrc ! videoconvert ! video/x-raw,format=I420,width=1920,height=1080,framerate=60/1 "
    //                                                                          "! omxh265enc bitrate=20000000 control-rate=2 "
    //                                                                          "! rtph265pay name=pay0 pt=96"), 60);
    // проверить есть ли разница по времени
    //   auto pipeLine =  QString("appsrc ! videoconvert ! "
    //                            "nvvidconv ! video/x-raw(memory:NVMM),format=I420,width=%1,height=%2,framerate=%3/1 ! "
    //                                                  "omxh265enc bitrate=200000000 iframeinterval=%4 preset-level=3 ! udpsink host=127.0.0.1 port=5000 sync=false async=false")
    //                              .arg(1920)
    //                              .arg(1080)
    //                              .arg(60)
    //                              .arg(15);
    //   qDebug () << pipeLine;
    //   setenv("GST_DEBUG","4", 0);
    //cv::VideoWriter out (pipeLine.toStdString(), cv::CAP_GSTREAMER, 0, 60, cv::Size(1920, 1080), true);
    //qDebug() << "try";
    //if (out.isOpened())
    //{
    //    qDebug() << "QQ";
    //}
}


void CameraClient::connectToCameraServer(const QHostAddress& address, quint16 port)
{
    sAddress = address;
    client->connectToHost(address, port);
}


void CameraClient::sendTest(const QByteArray& test)
{
    QByteArray bArray;
    bArray.append(GetTestDataFromClient);
    bArray.append(test);
    fillEndOfMessage(bArray);
    client->write(bArray);
}

void CameraClient::sendRecognizeVideo(const QLinkedList <FrameTime>& frames)
{

    RtspVideoHandlerParams params;
    params.cam = camera;
    params.w = camera->getWidth();
    params.h = camera->getHeight();
    params.framerate  = frameRateSend;
    params.rotate = camera->getRotate();
    collectData = 1;
    //    RtspVideoHandler handler(params, frames);
    sender->setRtspParams(params);
    sender->setVideoToSend(frames);
    sender->sendVideo(streamPort + 1, QString("appsrc name=vsrc ! videoconvert ! video/x-raw,format=I420,width=%1,height=%2,framerate=%3/1 "
                                              "! omxh265enc bitrate=20000000 control-rate=2 "
                                              "! rtph265pay name=pay0 pt=96")
                      .arg(camera->getWidth())
                      .arg(camera->getHeight())
                      .arg(frameRateSend));
    qDebug() << "FINISHED SERVER0";
    collectData = 0;
    qDebug() << "FINISHED SERVER";
}

void CameraClient::sendRecognizeResults(double ballCoordinates[][Camera::measureDim], qint32 frameCount, qint32 size)
{
    emit internalMessageReady("Create ball coordinates array.");
    qint32 sizeBallArray = Camera::measureDim * Camera::maxNumberOfMeasures * sizeof(double);
    QTime currentTime = QTime::currentTime();
    QByteArray bArray;
    bArray.append((char*)&currentTime, sizeof(QTime));
    bArray.append((char*)&frameCount, sizeof(qint32));
    memcpy(((char*)ballCoordinates + sizeBallArray - bArray.size()), bArray.data(), bArray.size());
    QByteArray array;
    array.append(GetThrowCoordinates);
    array.append((char*)ballCoordinates, sizeBallArray);
    array.append((char*)&size, sizeof(size));
    fillEndOfMessage(array);
    client->write(array);
    emit internalMessageReady("Coordinates sent");

}

void CameraClient::sendHitResults(double ballCoordinates[][Camera::measureDim], qint32 size)
{
    emit internalMessageReady("Create ball coordinates array.");
    qint32 sizeBallArray = Camera::measureDim * Camera::maxNumberOfMeasures * sizeof(double);
    QTime currentTime = QTime::currentTime();
    QByteArray bArray;
    bArray.append((char*)&currentTime, sizeof(QTime));
    memcpy(((char*)ballCoordinates + sizeBallArray - bArray.size()), bArray.data(), bArray.size());
    QByteArray array;
    array.append(GetHitCoordinates);
    array.append((char*)ballCoordinates, sizeBallArray);
    array.append((char*)&size, sizeof(size));
    fillEndOfMessage(array);
    client->write(array);
    emit internalMessageReady("Hit coordinates sent");
}



void CameraClient::handleError(QAbstractSocket::SocketError socketError)
{
    QString errorMessage;
    switch (socketError) {
    case QAbstractSocket::RemoteHostClosedError:
        errorMessage = "Remote host closed.";
        break;
    case QAbstractSocket::HostNotFoundError:
        errorMessage = "The host was not found. Please check the "
                       "host name and port settings.";
        break;
    case QAbstractSocket::ConnectionRefusedError:
        errorMessage = "The connection was refused by the peer. "
                       "Make sure the server is running, "
                       "and check that the host name and port "
                       "settings are correct.";
        break;
    default:
        errorMessage = QString("The following error occurred: %1.")
                .arg(client->errorString());
    }
    emit readyMessageFromClient(errorMessage);
}

void CameraClient::sendRawStream(qint32 count)
{
    qint32 avoidRefuseAttempts = 10;
    qint32 i = 0;
    bool success = false;
    while (i < avoidRefuseAttempts && !success)
    {
        QSharedPointer <QTcpSocket> imageSender(new QTcpSocket());
        imageSender->connectToHost(sAddress, streamPort);
        ++i;
        if (imageSender->waitForConnected(10000))
        {
            success = true;

            emit internalMessageReady(QString ("Подключение для передачи сырого стрима устанолено"));
            QSharedPointer <QMetaObject::Connection> conn (new QMetaObject::Connection);
            collectData = 1;

            *conn = connect(camera, &Camera::frameReady, imageSender.data(), [this, count, imageSender, conn](cv::Mat frame, QTime t, quint64 intT)
            {
                static qint32 i = 0;
                if (*conn)
                {
                    QByteArray array;
                    array.append((char*)&intT, sizeof(intT));
                    array.append((char*)&t, sizeof(t));
                    array.append((char*)frame.data, frame.cols * frame.rows);
                    frame.release();
                    qint32 size = array.size();
                    qint32 facticalSize = 0;
                    while ((facticalSize = imageSender->write(array)) != size)
                    {
                        array.remove(0, facticalSize);
                        size = size - facticalSize;
                    }
                    imageSender->flush();

                    ++i;
                    if ((count != -1 && i == count) || collectData == 0)
                    {
                        QObject::disconnect(*conn);
                        bool sendLeft = true;
                        while (sendLeft)
                        {
                            sendLeft = imageSender->flush();
                        }
                        emit internalMessageReady(QString ("%1 кадров успешно переданы.").arg(i));
                        i = 0;
                        collectData = 0;
                        emit sendFinished();

                    }
                }

            });
        }
        QThread::msleep(250);

    }
    if (!success)
    {
        emit sendFinished();
    }
}

void CameraClient::sendBallCoordinates(double array[][Camera::measureDim], qint32 size)
{
    QByteArray bArray;
    bArray.append(GetThrowCoordinates);
    memcpy(bArray.data(), &array, Camera::measureDim * Camera::maxNumberOfMeasures * sizeof(double));
    bArray.append(size);
    fillEndOfMessage(bArray);
    client->write(bArray);
}

void CameraClient::sendFrameInternal()
{
    QEventLoop loop;
    connect(this, &CameraClient::sendFinished, &loop, &QEventLoop::quit, Qt::QueuedConnection);

    sendRawStream(1);

    camera->tryToStartCamera();
    loop.exec();

}


void CameraClient::sendVideoInternal(qint32 frameCount)
{

    camera->tryToStartCamera();
    if (!rawTCP)
    {
        RtspVideoHandlerParams params;
        params.cam = camera;
        params.w = camera->getWidth();
        params.h = camera->getHeight();
        params.framerate  = frameRateSend;
        params.rotate = camera->getRotate();
        collectData = 1;
        RtspVideoHandler handler(params, this);

        //        handler.sendVideo(streamPort, QString("appsrc name=vsrc ! nvvidconv ! video/x-raw(memory:NVMM),format=I420,width=%1,height=%2,framerate=%3/1 ! "
        //                                              "omxh265enc bitrate=200000000 iframeinterval=%4 preset-level=3 ! rtph265pay name=pay0 pt=96")
        //                          .arg(camera->getWidth())
        //                          .arg(camera->getHeight())
        //                          .arg(frameRateSend)
        //                          .arg(frameRateSend / 4));

//        handler.sendVideo(streamPort, QString("appsrc name=vsrc ! videoconvert ! video/x-raw,format=I420,width=%1,height=%2,framerate=%3/1 "
//                                              "! omxh265enc MeasureEncoderLatency=true bitrate=20000000 control-rate=2 "
//                                              "! rtph265pay name=pay0 pt=96")
//                          .arg(camera->getWidth())
//                          .arg(camera->getHeight())
//                          .arg(frameRateSend), frameCount);
        handler.sendVideo(streamPort, QString("( appsrc name=vsrc ! videoconvert ! video/x-raw,format=RGBA, width=%1,height=%2,framerate=%3/1 "
                                              "! nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! omxh265enc MeasureEncoderLatency=true bitrate=20000000 control-rate=2 "
                                              "! rtph265pay name=pay0 pt=96 )")
                          .arg(camera->getWidth())
                          .arg(camera->getHeight())
                          .arg(frameRateSend), frameCount);
        collectData = 0;

        //        handler.sendVideo(streamPort, QString("appsrc name=vsrc ! videoconvert ! video/x-raw,format=I420,width=%1,height=%2,framerate=%3/1 ! "
        //                                              "omxh265enc bitrate=20000000 control-rate=2 iframeinterval=%4 preset-level=3 "
        //                                              "! rtph265pay name=pay0 pt=96")
        //                          .arg(camera->getWidth())
        //                          .arg(camera->getHeight())
        //                          .arg(frameRateSend)
        //                          .arg(frameRateSend / 2));
        qDebug() << "FINISHED SERVER";
        //        QString pipeLine = QString("appsrc ! videoconvert ! video/x-raw,format=I420,width=%1,height=%2,framerate=%3/1 !  "
        //                                   "omxh265enc bitrate=40000000 iframeinterval=%4 preset-level=3 vbv-size=1 "
        //                                   "EnableTwopassCBR=true EnableStringentBitrate=true ! matroskamux "
        //                                   " ! filesink location=/home/nvidia/actual_client/test_exp/test5.mkv")
        //                .arg(camera->getWidth())
        //                .arg(camera->getHeight())
        //                .arg(frameRateSend)
        //                .arg(frameRateSend / 2);
        //        qDebug() << pipeLine;
        //        QSharedPointer<cv::VideoWriter> out (new cv::VideoWriter(pipeLine.toStdString(), cv::CAP_GSTREAMER, 0, frameRateSend, cv::Size(camera->getWidth(), camera->getHeight()), true));
        //        collectData = 1;
        //        if (out->isOpened())
        //        {
        //            emit internalMessageReady(QString ("Подключение для передачи сырого стрима устанолено"));
        //            QSharedPointer <QMetaObject::Connection> conn (new QMetaObject::Connection);
        //            *conn = connect(camera, &Camera::frameReady, [out, this, frameCount, conn](auto frame, QTime t, quint64 intT) mutable
        //            {
        //                static qint32 i = 0;
        //                Q_UNUSED(t);
        //                Q_UNUSED(intT);
        //                if (collectData == 0)
        //                {
        //                    emit this->internalMessageReady(QString ("%1 кадров успешно переданы.").arg(i));
        //                    emit this->sendFinished();
        //                    disconnect(*conn);
        //                    out.clear();
        //                    i = 0;
        //                }
        //                else
        //                {
        //                    Mat fr;
        //                    cvtColor(frame, fr, CV_BayerBG2BGR);
        //                    qDebug() << i << "write";
        //                    out->write(fr);
        //                    ++i;
        //                }

        //});
        //}
    }
    else
    {
        QEventLoop loop;
        connect(this, &CameraClient::sendFinished, &loop, &QEventLoop::quit);
        sendRawStream(frameCount);
        loop.exec();
    }


}

void CameraClient::fillEndOfMessage(QByteArray& array)
{
    for (qint32 i = 0; i < 10; ++i)
    {
        array.append(EndOfMessageSign);
    }

}

void CameraClient::sendCurrentCameraParamsInternal()
{
    QByteArray array;
    camera->fillCurrentCameraParameters();
    CurrentCameraParams params = camera->getCurrentCameraParameters();
    params.sendFrameRate = frameRateSend;
    params.portSendStream = streamPort;
    array.append(AskCurrentCameraParams);
    array.append((const char*)&params, sizeof(CurrentCameraParams));
    fillEndOfMessage(array);
    client->write(array);
}

void CameraClient::sendCurrentTimeInternal(quint64 t)
{
    QByteArray array;
    array.append(SendCurrentFrameTime);
    array.append((const char*)&t, sizeof(quint64));
    fillEndOfMessage(array);
    client->write(array);
}

void CameraClient::handleMessageFromServer()
{

    bool read = true;
    QByteArray endOfMessage;
    fillEndOfMessage(endOfMessage);
    while (read)
    {
        data.append(client->readAll());
        qint32 pos = data.indexOf(endOfMessage);
        if (avaliableCommands.contains(data[0]) &&
                pos != -1)
        {
            qint16 flag = data[0];
            qint32 size = pos - 1;
            auto clearData = data.remove(0, 1).remove(pos - 1, endOfMessage.size());
            switch (flag)
            {
            case SendCameraParams:
            {
                CameraOptions params;
                memmove(&params, clearData.data(), sizeof(CameraOptions));
                if (!qFuzzyCompare(params.frameRateSend, -1.0))
                {
                    frameRateSend = params.frameRateSend;
                }
                if (params.portSendStream != -1)
                {
                    streamPort = params.portSendStream;
                }
                if (params.rawFrame != -1)
                {
                    rawTCP = params.rawFrame;
                }
                camera->assignCameraOptions(params);
                break;
            }
            case RequestCameraFrame:
            {
                if (collectData == 0)
                {
                    QtConcurrent::run(this, &CameraClient::sendFrameInternal);
                }
                else
                {
                    QString ("Камера работает с предыдущим запросом. Нажмите \"Остановить стрим\" чтобы прервать запрос.");
                }

                break;
            }
            case RequestCameraVideo:
            {
                if (collectData == 0)
                {
                    qint32 frameCount = camera->getVideoDuration() * camera->getFrameRate();
                    emit internalMessageReady(QString("Запрошено %1 кадров.").arg(frameCount));
                    QtConcurrent::run(this, &CameraClient::sendVideoInternal, frameCount);
                }
                else
                {
                    emit internalMessageReady(QString ("Камера работает с предыдущим запросом. Нажмите \"Остановить стрим\" чтобы прервать запрос."));
                }
                break;
            }
            case StopStream:
            {
                collectData = 0;
                break;
            }
            case StartStream:
            {
                QtConcurrent::run(this, &CameraClient::sendVideoInternal, -1);
                break;
            }

            case RestartCamera:
            {
                emit internalMessageReady(QString ("Камера остановлена."));
                camera->stopLiveVideo();
                camera->startLiveVideo();
                break;
            }
            case AskCurrentCameraParams:
            {
                sendCurrentCameraParamsInternal();
                break;
            }
            case SendSaveParameters:
            {
                saveSettings();
                camera->saveSettings();
                emit internalMessageReady(QString ("Настройки успешно сохранены."));
                break;
            }


            }
            data.remove(0, size);
        }
        else
        {
            read = false;
        }
    }
}

CameraClient::~CameraClient()
{
    client->disconnectFromHost();
    saveSettings();
}

void CameraClient::loadSettings()
{
    QSettings settings;
    streamPort = settings.value("send_port", 5000).toInt();
    frameRateSend = settings.value("frame_rate_send").toDouble();
    rawTCP = settings.value("raw_frame").toBool();
}

void CameraClient::saveSettings()
{
    QSettings settings;
    settings.setValue("send_port", streamPort);
    settings.setValue("frame_rate_send", frameRateSend);
    settings.setValue("raw_frame", rawTCP);
    settings.sync();

}


