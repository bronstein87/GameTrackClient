#include "camera.h"

Camera::Camera(const QString& pattern, bool init, qint32 port, QObject* parent) :
    QObject(parent), controller(pattern, port)/*, batTracker(1920, 1080)*/
{
    loadSettings();
    if (!options.has_id())
    {
        options.mutable_hw_params()->set_width(1920);
        options.mutable_hw_params()->set_height(1080);
        options.mutable_hw_params()->set_frame_rate(60);
        options.mutable_hw_params()->set_pixel_clock(474);
        options.set_auto_exposure_enable(true);
        options.mutable_auto_exp_params()->set_gain(30);
        options.mutable_auto_exp_params()->set_min_gain_coeff(30);
        options.mutable_auto_exp_params()->set_max_gain_coeff(80);
        options.mutable_auto_exp_params()->set_max_percent(0.05);
        options.mutable_auto_exp_params()->set_min_rel_coef(0.9);
        options.mutable_auto_exp_params()->set_max_rel_coef(1.1);
        options.mutable_stream_params()->set_send_frame_rate_main(60);
        options.mutable_stream_params()->set_send_frame_rate_add(180);
        options.set_main_add_mode(false);
        options.set_main_each(3);
        options.mutable_rec_params()->set_corr_coef(0.7);
        options.mutable_rec_params()->set_min_area(400);
        options.mutable_rec_params()->set_max_area(800);
        options.mutable_rec_params()->set_min_speed(15);
        options.mutable_rec_params()->set_sko_coef(2.5);
        options.mutable_rec_params()->set_max_angle_directions(18);
        options.mutable_rec_params()->set_search_area_size(200);
        options.mutable_rec_params()->set_canny_thres_min(50);
        options.mutable_rec_params()->set_canny_thres_max(150);
        options.mutable_rec_params()->set_circularity_coeff(0.8);

        options.mutable_rec_rois()->mutable_throw_search_rect()->mutable_xy()->set_x(20);
        options.mutable_rec_rois()->mutable_throw_search_rect()->mutable_xy()->set_y(240);
        options.mutable_rec_rois()->mutable_throw_search_rect()->mutable_wh()->set_x(800);
        options.mutable_rec_rois()->mutable_throw_search_rect()->mutable_wh()->set_y(600);
        options.mutable_rec_rois()->mutable_throw_track_rect()->CopyFrom(*options.mutable_rec_rois()->mutable_throw_search_rect());
        options.mutable_rec_rois()->mutable_hit_search_rect()->CopyFrom(*options.mutable_rec_rois()->mutable_throw_search_rect());
    }


    if (init)
    {
        if (!options.has_debug_mode())
        {
            options.set_debug_mode(false);
        }
        //1936, 1216
        initializeCameraInterface();
        setHWParams(options, true);
        fillCurrentCameraParameters();

        QFile file;
        file.setFileName("config/config.json");
        file.open(QIODevice::ReadOnly | QIODevice::Text);
        QJsonDocument staticParameters = QJsonDocument::fromJson(file.readAll());
        auto data = staticParameters.object();
        file.close();
        options.set_id(data["id"].toString().toStdString());
        options.set_cam_type((CameraType)data["type"].toInt());
        options.mutable_stream_params()->set_port_send_stream_main(data["port1"].toInt());
        options.mutable_stream_params()->set_port_send_stream_add(data["port2"].toInt());
        debugVideoPath = data["debug_path"].toString();
        qDebug()<< options.mutable_stream_params()->port_send_stream_main()
                << options.mutable_stream_params()->port_send_stream_add();
        CalibrationHelper::instance().setCurrentCamera(QString::fromStdString(options.id()));
        autoExpHandler.setParameters(options.mutable_auto_exp_params());
        recognizer.setROIs(options.mutable_rec_rois());
        recognizer.setRecognizeParams(options.mutable_rec_params());
        connect(&autoExpHandler, &AutoExposureHandler::currentStateReady, this, &Camera::messageFromCameraReady);
        tryToStartCamera();
    }

//    if (options.cam_type() > CameraType::BaseRightAdd)
//    {
//        batTracker.initCNN("/home/nvidia/Downloads/net/deploy.prototxt", "/home/nvidia/Downloads/net/snapshot_iter_340400.caffemodel", 0.6);
//        batTracker.setBallRecognizer(&recognizer);
//    }
//    else
//    {
//        pedTracker.initCNN(detectNet::PEDNET, 0.5);
//        recognizer.setPedestrianTracker(&pedTracker);
//    }

    connect(&recognizer, &BallRecognizerPP::ballRecognized, this, &Camera::ballMeasureReady);
    connect(&recognizer, &BallRecognizerPP::ballOutOfFrame, this, &Camera::ballOutOfFrame);

    //    QtConcurrent::run([]()
    //    {
    //        while(true)
    //        {
    //            qDebug() << "FFFFFF";
    //        }
    //    });
    //    QtConcurrent::run([]()
    //    {
    //        while(true)
    //        {
    //            qDebug() << "BBBBBb";
    //        }
    //    });
}



void Camera::setTriggerModeEnable(bool enable)
{
    options.mutable_hw_params()->set_trigger_mode_enable(enable);
    stopLiveVideo();
    qint32 errorCode;
    qint32 mode;

    if (!options.hw_params().trigger_mode_enable())
    {
        mode = IS_SET_TRIGGER_OFF;
    }
    else
    {
        mode = options.hw_params().trigger_mode();
    }

    if ((errorCode = is_SetExternalTrigger(hCam, mode)) != IS_SUCCESS)
    {
        QString error = "Не удалось установить триггерный режим, код ошибки: " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        startLiveVideo();
        qDebug() << "trigger" << mode;
    }


}

void Camera::setTriggerMode(qint32 mode)
{
    options.mutable_hw_params()->set_trigger_mode(mode);
}

void Camera::tryToStartCamera()
{
    if (!streamIsActive)
    {

            QtConcurrent::run(this, &Camera::procImageQueue);
            QThread::msleep(250);

#ifdef AUTOEXP_MAIN_THREAD
        if (!options.has_debug_mode() || !options.debug_mode())
        {
            doAutoExposure();
        }
#endif

    }
}



void Camera::procImageQueue()
{
    if (streamIsActive)
    {
        return;
    }
    streamIsActive = 1;
#ifndef AUTOEXP_MAIN_THREAD
    QtConcurrent::run(this, &Camera::doAutoExposure);
#endif

    qint32 i = 0;
    qint32 nMemID = 0;
    char* pBuffer = nullptr;
    qint32 nRet;
    static qint64 timeStampPrevious;
    static QTime timestampSystemPrevious;
    timestampSystemPrevious = QTime();
    timeStampPrevious = -1;
    do
    {
        QElapsedTimer tlag;

        tlag.start();
        nRet = is_WaitForNextImage(hCam, 1000, &pBuffer, &nMemID);
        if (nRet == IS_SUCCESS)
        {
            QDateTime dt = QDateTime::currentDateTime();
            quint64 tLagElapsed = tlag.elapsed();
            UEYEIMAGEINFO imageInfo;
            nRet = is_GetImageInfo(hCam, nMemID, &imageInfo, sizeof(imageInfo));

            if (nRet == IS_SUCCESS)
            {

                QTime timestampSystem = QTime(imageInfo.TimestampSystem.wHour,imageInfo.TimestampSystem.wMinute,
                                              imageInfo.TimestampSystem.wSecond, imageInfo.TimestampSystem.wMilliseconds);
                 //qDebug() << imageInfo.u64TimestampDevice << timestampSystem << options.hw_params().frame_rate();

                qint64 diff = abs(timeStampPrevious - (qint64)imageInfo.u64TimestampDevice);
                qint64 thres = (1. / options.hw_params().frame_rate()) * 1e7 + 1e4;

                if (timeStampPrevious != -1 && diff > thres)
                {
                    bool fixed = false;
                    qint32 delta = abs(timestampSystemPrevious.msecsTo(timestampSystem));
                    if (delta > delayTreshold)
                    {
                        imageInfo.u64TimestampDevice = timeStampPrevious + thres;
                        fixed = true;
                    }
                    QString delayMessage = QString("Delay. Await time: %1. Camera time : %2. System time from camera: %3."
                                                   "System time from computer: %4. Diff with previous time %5."
                                                   "Previous camera time: %6. Fixed: %7")
                            .arg(tLagElapsed)
                            .arg(imageInfo.u64TimestampDevice)
                            .arg(timestampSystem.toString())
                            .arg(dt.toString())
                            .arg(diff)
                            .arg(timeStampPrevious)
                            .arg(fixed);
                    qDebug() << delayMessage;
                    emit this->messageFromCameraReady(delayMessage);
                }
                emit currentTimeReady(imageInfo.u64TimestampDevice, timestampSystem);
                FrameInfo ft;
                ft.time = timeStampPrevious = imageInfo.u64TimestampDevice;
                ft.computerTime = timestampSystem.msecsSinceStartOfDay();
                ft.computerTimeTest = dt.toMSecsSinceEpoch();
                timestampSystemPrevious = timestampSystem;
                ft.frame = Mat(options.hw_params().height(), options.hw_params().width(), CV_8UC1, pBuffer);
                ft.memoryId = nMemID;

                QMutexLocker lock(&recMutex);
                if (options.main_add_mode())
                {
                    if (i % options.main_each() == 0)
                    {
                        ft.mainFrame = true;
                    }
                }
                else
                {
                    ft.mainFrame = true;
                }
                bufferFrames.append(ft);
                if (bufferFrames.size() >= maxBufferSize - 1)
                {
                    //qDebug() << "clear";
                    is_UnlockSeqBuf (hCam, bufferFrames.first().memoryId, (char*)bufferFrames.first().frame.data);
                    bufferFrames.removeFirst();
                }

                lock.unlock();

                emit frameReady(bufferFrames.last());
                ++i;
            }
            else
            {
                qDebug() << "CAN'T GET FRAME" << nRet;
            }
        }
    }
    while (streamIsActive);
    bufferFrames.clear();
}

//void Camera::procImageImitation()
//{
//    qDebug() << "open debug" << debugVideoPath;
//    VideoCapture capture = VideoCapture(debugVideoPath.toStdString());
//    Mat m;
//    //QFile file(debugTimePath);
//    //file.open(QIODevice::ReadOnly);
//    //QTextStream in (&file);

//    if (capture.isOpened())
//    {
//        qDebug() << "record is opened";
//        qint32 skip = 0;
//        qint32 frameNumber = 1;
//        qint32 delta = (1. / options.hw_params().frame_rate()) * 1e7;
//        constexpr const qint32 takeEach = 3;
//        bool  isDebuging = true;
//        while (isDebuging)
//        {
//            if (!waitForCommand)
//            {
//                if (moveFrames != 0)
//                {
//                    capture.set(CAP_PROP_POS_FRAMES, capture.get(CAP_PROP_POS_FRAMES) + moveFrames);
//                    pedTracker.clear();
//                    moveFrames = 0;
//                }
//                isDebuging = true;
//                if (isDebuging)
//                {
//                    while (skip < takeEach)
//                    {
//                        isDebuging = capture.read((m));
//                        ++skip;
//                    }
//                    skip = 0;
//                    FrameInfo ft;
//                    m.copyTo(ft.frame);


//                    pedTracker.track(ft.frame, ft.time);
//                    qDebug() << "draw";
//                    pedTracker.drawROIs(ft.frame);
//                    ft.time = delta * frameNumber;
//                    ++frameNumber;
//                    qDebug() << "push";
//                    ft.mainFrame = true;
//                    bufferFrames.append(ft);
//                    waitForCommand = true;


//                    if (bufferFrames.size() >= 100)
//                    {
//                        bufferFrames.removeFirst();
//                    }

//                }

//            }

//        }
//    }
//}


void Camera::startRecognition()
{
    QtConcurrent::run(this, &Camera::recognitionProcInternal);
    QtConcurrent::run(this, &Camera::procImageQueue);
    QThread::msleep(250);

#ifdef AUTOEXP_MAIN_THREAD
    doAutoExposure();
#endif
}

void Camera::recognitionProcInternal()
{
    constexpr const static qint32 keepFrames = 180;
    if (!options.ball_recognize_enable())
    {
        options.set_ball_recognize_enable(true);
        qint32 skipCounter = 0;
        Mat frame;
        QLinkedList <FrameInfo>::iterator it;
        auto conn = connect(&recognizer, &BallRecognizerPP::resultsReady, this, [this, &it](auto state, auto result) mutable
        {
            if (state == BallRecognizerPP::BallThrow)
            {
                // добавить проверку направления путем получения 3d координат через Z=0 для первой и последней точки
                // измерения расстояния до дома (только для УПРОЩЕННОГО режима)
                QMutexLocker lock(&recMutex);
                QLinkedList <FrameInfo>::iterator itcur = bufferFrames.begin();
                QVector <FrameInfo> prev;

                itcur = bufferFrames.begin();
                while (itcur != bufferFrames.end())
                {
                    ++itcur;
                    if (itcur->time == result.meta.first().time)
                    {
                        auto itprev = itcur;
                        const qint32 maxPrevFrames = 30;
                        while (itprev != bufferFrames.begin() && prev.size() < maxPrevFrames)
                        {
                            --itprev;
                            prev.append(*itprev);
                        }
                        break;
                    }
                }
                qint32 facticalRecCount = 0;
                recognizer.analyzePreviousFrames(result, prev);
                msg::RecognizeData recData;
                for (qint32 i = 0; i < result.arrows.size(); ++i)
                {
                    auto measure = recData.mutable_data()->Add();
                    measure->mutable_xy()->set_x(result.arrows[i].second.x);
                    measure->mutable_xy()->set_y(result.arrows[i].second.y);
                    measure->set_valid(result.meta[i].valid);
                    measure->set_event(msg::BallEvent::ThrowDetected);
                    if (measure->valid())
                    {
                        ++facticalRecCount;
                    }
                    measure->set_is_rebound(result.meta[i].rebound);
                    measure->set_time((double)result.meta[i].time / 10000000.0);
                }

                qint32 size = result.meta.size();
                while (it->time != result.meta[size - 1].time)
                {
                    --it;
                }
                // пропускаем все кадры, которые в/после пересечении с бьющим
                qint32 startFrame = 0;
                for (startFrame = result.meta.size() - 1; startFrame > 0; --startFrame)
                {
                    // если измерение невалидно или последующее за ним невалидно (т.е мяч скорее всего пролетает через/возле бьющего
                    if (!result.meta[startFrame].valid
                            || (result.meta[startFrame].valid
                                && !result.meta[startFrame - 1].valid))
                    {
                        --it;
                    }
                    else
                    {
                        break;
                    }
                }

                const qint32 skipFrames = 3;
                for (qint32 i = 0; i < skipFrames; ++i)
                {
                    --it;
                }

                auto startTrackHitPreviousIt = it;
                auto startTrackHitIt = startTrackHitPreviousIt;
                ++startTrackHitIt;
                startFrame -= skipFrames;
                Q_ASSERT(startTrackHitPreviousIt->time == result.meta[startFrame].time);

                // вырезаем мяч с кадра начала распознавания удара
                Mat fb;
                startTrackHitIt->frame.copyTo(fb);
                auto bCenter = result.arrows[startFrame].second;
                const qint32 areaSize = 30;
                auto rect = cv::Rect(bCenter.x - areaSize / 2, bCenter.y - areaSize / 2, areaSize, areaSize);
                startTrackHitPreviousIt->frame(rect).copyTo(fb(rect));

                cvtColor(fb, fb, CV_BayerBG2GRAY);
                recognizer.setBackGroundFrame(fb);
                lock.unlock();

                emit this->messageFromCameraReady(QString("BALL RECOGNIZED %1 TIMES %2")
                                                  .arg(facticalRecCount)
                                                  .arg(bufferFrames.size()));
                emit this->ballCoordinatesReady(recData);
            }
            else if (state == BallRecognizerPP::BallHit)
            {
                qint32 facticalRecCount = 0;
                msg::RecognizeData recData;
                for (qint32 i = 0; i < result.arrows.size(); ++i)
                {
                    auto measure = recData.mutable_data()->Add();
                    measure->mutable_xy()->set_x(result.arrows[i].second.x);
                    measure->mutable_xy()->set_y(result.arrows[i].second.y);
                    measure->set_valid(result.meta[i].valid);
                    measure->set_event(msg::BallEvent::HitDetected);
                    if (measure->valid())
                    {
                        ++facticalRecCount;
                    }
                    measure->set_is_rebound(result.meta[i].rebound);
                    measure->set_time((double)result.meta[i].time / 10000000.0);
                }
                emit this->messageFromCameraReady(QString("BALL RECOGNIZED %1 TIMES %2")
                                                  .arg(facticalRecCount)
                                                  .arg(bufferFrames.size()));
                emit this->ballCoordinatesReady(recData);

            }
            //send everything
        });


        while (options.ball_recognize_enable())
        {
            QMutexLocker lock(&recMutex);
            if (!bufferFrames.isEmpty())
            {
                if (it == QLinkedList <FrameInfo>::iterator())
                {
                    it = bufferFrames.begin();
                }

                if (++it != bufferFrames.end())
                {
                    ++skipCounter;
                }
                else
                {
                    --it;
                }
                lock.unlock();
                if (skipCounter == std::round(options.hw_params().frame_rate()) / 60)
                {
                    skipCounter = 0;
                    frame = it->frame;

                    Mat cvtMat;
                    // recognizer.setDebugMessage("CVT");
                    cvtColor(frame, cvtMat, CV_BayerBG2GRAY);
                    // recognizer.setDebugMessage("START");
                    BallRecognizerPP::State state = recognizer.recognize(cvtMat, it->time);

                    //recognizer.setDebugMessage("OBR OCH");
                    QMutexLocker lock(&recMutex);
                    if (state == BallRecognizerPP::BallThrow) // запоминаем, чтобы обработать незахваченные сначала кадры
                    {
                        // recognizer.setDebugMessage("OCH OCR");
                        qint32 i = 0;
                        auto prevIt = it;
                        while (--prevIt != bufferFrames.begin() && i != keepFrames)
                        {
                            ++i;
                        }
                        if (i == keepFrames)
                        {
                            bufferFrames.erase(bufferFrames.begin(), prevIt);
                        }

                        //recognizer.setDebugMessage("ZAK OBR OCH");

                    }
                }
            }
            //recognizer.setDebugMessage("FIN");
            bufferFrames.clear();
            disconnect(conn);
        }
    }
}


void Camera::startLiveVideo()
{
    qint32 errorCode;
    if (!is_CaptureVideo(hCam, IS_GET_LIVE))
    {
        if (!camSeqBuild() || (errorCode = is_CaptureVideo(hCam, IS_DONT_WAIT)) != IS_SUCCESS)
        {
            QString error = "не удалось начать съемку, код ошибки:" + QString::number(errorCode);
            qDebug() << error;
            emit messageFromCameraReady(error);
        }
    }
    else
    {
        QString error = "Live mode already enabled";
        qDebug() << error;
        emit messageFromCameraReady(error);
    }

}

void Camera::stopLiveVideo()
{
    qint32 errorCode;
    streamIsActive = 0;
    if (camSeqKill() &&
            ((errorCode = is_StopLiveVideo(hCam, IS_FORCE_VIDEO_STOP)) != IS_SUCCESS))
    {
        QString error = "не удалось закончить съемку, код ошибки:" + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
}


void Camera::unlockAllBuffer()
{
    qint32 errorCode = -1;
    for (qint32 i = (maxBufferSize - 1); i >= 0; --i)
    {
        // free buffers
        if((errorCode = is_UnlockSeqBuf(hCam, m_viSeqMemId.at(i), m_vpcSeqImgMem.at(i))) != IS_SUCCESS )
        {
            QString error = "не удалось освободить буфер: " + QString::number(errorCode);
            qDebug() << error;
            emit messageFromCameraReady(error);
        }
    }
}

void Camera::setROI(IS_SIZE_2D size)
{
    qint32 errorCode;
    if (streamIsActive)
    {
        streamIsActive = 0;
        stopLiveVideo();
    }
    if ((errorCode = is_AOI(hCam, IS_AOI_IMAGE_SET_SIZE, (void*)&size, sizeof(size))) != IS_SUCCESS)
    {
        QString error = "не удалось изменить область интереса, код ошибки: " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
}

void Camera::setFocusing(const QString& value)
{
    QString tmp = value;
    tmp.remove(0, 1).remove(tmp.size() - 1, 1);
    options.mutable_hw_params()->set_focusing(tmp.toInt());
    controller.setFocusing(value);
}

void Camera::setDiaphraghmLevel(const QString& value)
{
    controller.setDiaphragmLevel(value);
}

QString Camera::getFocusing()
{
    return controller.getCurrentFocusing();
}


void Camera::setFrameRate(qint32 fps)
{
    qint32 errorCode;
    double frameRateTmp;
    if ((errorCode = is_SetFrameRate(hCam, fps, &frameRateTmp))  != IS_SUCCESS)
    {
        QString error = "не удалось установить новый фреймрейт, код ошибки: " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        if (qFuzzyCompare(frameRateTmp, 0))
            options.mutable_hw_params()->set_frame_rate(std::round(frameRateTmp));
    }
    qDebug() << "NEW FPS: " << options.mutable_hw_params()->frame_rate();

}

void  Camera::setPixelClock(qint32 pixelClock)
{
    qint32 errorCode;
    if ((errorCode = is_PixelClock(hCam, IS_PIXELCLOCK_CMD_SET, (void*)&pixelClock, sizeof(pixelClock))) !=
            IS_SUCCESS)
    {
        QString error = "не удалось установить новый пиксел клок, код ошибки: " + QString::number(errorCode) +
                QString("(%1)").arg(pixelClock);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
}

void Camera::setExposure(double exposure)
{
    qint32 errorCode;

    QElapsedTimer t;
    t.start();
    if (options.mutable_hw_params()->min_exposure() > exposure || exposure > options.mutable_hw_params()->max_exposure())
    {
        options.mutable_auto_exp_params()->set_exposure(options.mutable_hw_params()->exposure());
    }
    else if ((errorCode = is_Exposure(hCam, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&exposure, sizeof(exposure))) !=
            IS_SUCCESS)
    {
        QString error = "не удалось установить новую экспозицию, код ошибки: " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        options.mutable_auto_exp_params()->set_exposure(exposure);
        options.mutable_hw_params()->set_exposure(exposure);
    }
}

void Camera::setHWParams(const msg::CameraOptions& _options, bool initialize = false)
{
    auto hw_params = _options.hw_params();

    if (hw_params.has_trigger_mode())
    {
        setTriggerMode(hw_params.trigger_mode());
    }

    if (hw_params.has_trigger_mode_enable())
    {
        setTriggerModeEnable(hw_params.trigger_mode_enable());
    }

    if (hw_params.has_pixel_clock())
    {
        setPixelClock(hw_params.pixel_clock());
    }
    if (hw_params.has_exposure())
    {
        setExposure(hw_params.exposure());
    }
    if (hw_params.has_focusing() && !initialize)
    {
        QString tmpValue = QString::number(hw_params.focusing());
        while (tmpValue.size() < 4)
        {
            tmpValue.prepend("0");
        }
        QString focusing = "M" + tmpValue + "#";
        setFocusing(focusing);
    }
    if (hw_params.has_frame_rate())
    {
        setFrameRate(hw_params.frame_rate());
    }

    if (hw_params.has_debounce_value())
    {
        setDebounceValue(hw_params.debounce_value());
    }

    if (hw_params.has_debounce_enable())
    {
        setDebounceEnable(hw_params.debounce_enable());
    }

    if (hw_params.has_gain())
    {
        setGain(hw_params.gain());
    }

    updateHWParameters();

    msg::CameraOptions opt;
    opt.mutable_hw_params()->CopyFrom(options.hw_params());
    opt.set_id(options.id());
    opt.set_main_add_mode(options.main_add_mode());

    emit parametersChanged(opt);
}

void Camera::assignCameraOptions(const msg::CameraOptions& _options)
{
    QMutexLocker lock(&optionMutex);

    if (_options.has_auto_exposure_enable())
    {
        setAutoExposure(_options.auto_exposure_enable());
    }


    if (_options.has_ball_recognize_enable())
    {
        setBallRecognizeFlag(_options.ball_recognize_enable());
    }

    if (_options.has_debug_enable())
    {
        setBallRecognizeFlagDebug(_options.debug_enable());
    }

    if (_options.has_debug_mode())
    {
        options.set_debug_mode(_options.debug_mode());
    }

    if (_options.has_hw_params())
    {
        setHWParams(_options);
    }

    if (_options.has_p_params())
    {
        options.mutable_p_params()->MergeFrom(_options.p_params());
    }
    if (_options.has_calib_params())
    {
        options.mutable_calib_params()->MergeFrom(_options.calib_params());
        auto& calibHelper = CalibrationHelper::instance();
        Calibration::SpacecraftPlatform::CAMERA::CameraParams Camera;
        Calibration::ExteriorOr EO;
        calibHelper.convertFromProto(_options.calib_params(), Camera, EO);
        //fix
        Camera.x_direction = Calibration::SpacecraftPlatform::CAMERA::CameraXdirection::right;
        Camera.z_direction = Calibration::SpacecraftPlatform::CAMERA::CameraZdirection::frompage;
        Camera.cameraType = Calibration::SpacecraftPlatform::CAMERA::CameraType::central;
        calibHelper.setEO(QString::fromStdString(options.id()), EO);
        calibHelper.setCameraParams(QString::fromStdString(options.id()), Camera);
    }
    if (_options.has_rec_params())
    {
        options.mutable_rec_params()->MergeFrom(_options.rec_params());
    }
    if (_options.has_rec_rois())
    {
        options.mutable_rec_rois()->MergeFrom(_options.rec_rois());
    }
    if (_options.has_stream_params())
    {
        options.mutable_stream_params()->MergeFrom(_options.stream_params());
    }
    if (_options.has_auto_exp_params())
    {
        options.mutable_auto_exp_params()->MergeFrom(_options.auto_exp_params());
        autoExpHandler.setParameters(options.mutable_auto_exp_params());
    }

    if (_options.has_save_parameters())
    {
        options.set_save_parameters(_options.save_parameters());
    }
    if (options.save_parameters())
    {
        saveSettings();
    }
}



void Camera::getHWExposure(qint32 errorCode)
{
    double exposureTmp;
    if ((errorCode = is_Exposure(hCam, IS_EXPOSURE_CMD_GET_EXPOSURE, (void*)&(exposureTmp),
                                 sizeof(exposureTmp))) != IS_SUCCESS)
    {
        QString error = "не удалось получить экспозицию, код ошибки: " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        options.mutable_auto_exp_params()->set_exposure(exposureTmp);
        options.mutable_hw_params()->set_exposure(exposureTmp);
    }
}

void Camera::getHWExposureRange(qint32 errorCode)
{
    double exposureMinTmp;
    if ((errorCode = is_Exposure(hCam, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MIN,
                                 (void*)&(exposureMinTmp), sizeof(exposureMinTmp))) != IS_SUCCESS)
    {
        QString error = "не удалось получить мин. экспозицию, код ошибки: " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        options.mutable_hw_params()->set_min_exposure(exposureMinTmp);
    }


    double exposureMaxTmp;
    if ((errorCode = is_Exposure(hCam, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MAX,
                                 (void*)&(exposureMaxTmp), sizeof(exposureMaxTmp))) != IS_SUCCESS)
    {
        QString error = "не удалось получить макс. экспозицию, код ошибки: " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        options.mutable_hw_params()->set_max_exposure(exposureMaxTmp);
    }
}

void Camera::getHWPixelClock(qint32 errorCode)
{
    qint32 pixelClockTmp;
    if ((errorCode = is_PixelClock(hCam, IS_PIXELCLOCK_CMD_GET, (void*)&pixelClockTmp,
                                   sizeof(pixelClockTmp))) != IS_SUCCESS)
    {
        QString error = "не удалось установить новый пиксел клок, код ошибки: " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        options.mutable_hw_params()->set_pixel_clock(pixelClockTmp);
    }
}

void Camera::getHWPixelClockRange(qint32 errorCode)
{
    UINT nRange[3];
    if ((errorCode = is_PixelClock(hCam, IS_PIXELCLOCK_CMD_GET_RANGE, (void*)nRange, sizeof(nRange))) !=
            IS_SUCCESS)
    {
        QString error = "не удалось получить ренж пикселклока, код ошибки: " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        options.mutable_hw_params()->set_min_pixel_clock(nRange[0]);
        options.mutable_hw_params()->set_max_pixel_clock(nRange[1]);
    }
}

void Camera::getHWFrameTimeRange(qint32 errorCode)
{
    double dummy, minFrameRateTmp, maxFrameRateTmp;
    if ((errorCode = is_GetFrameTimeRange(hCam, &minFrameRateTmp,
                                          &maxFrameRateTmp, &dummy)) != IS_SUCCESS)
    {
        QString error = "Не удалось получить рендж фреймрета " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        double tmp = maxFrameRateTmp;
        options.mutable_hw_params()->set_max_frame_rate(1. / minFrameRateTmp);
        options.mutable_hw_params()->set_min_frame_rate(1. / tmp);
    }
}

void Camera::getHWFrameTime(qint32 errorCode)
{
    double frameRateTmp;
    if ((errorCode = is_GetFramesPerSecond (hCam, &frameRateTmp)) != IS_SUCCESS)
    {
        QString error = "Не удалось получить текущий фреймрейт" + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        if (!qFuzzyCompare(frameRateTmp, 0))
            options.mutable_hw_params()->set_frame_rate(std::round(frameRateTmp));
    }
}

void Camera::getHWTriggerDebounce(qint32 errorCode)
{
    qint32 debounceEnableTmp;
    if ((errorCode = is_TriggerDebounce(hCam, TRIGGER_DEBOUNCE_CMD_GET_MODE, &debounceEnableTmp, sizeof(qint32))) != IS_SUCCESS)
    {
        QString error = "Не удалось получить режим дребезжания триггера" + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        options.mutable_hw_params()->set_debounce_enable(debounceEnableTmp);
    }
}

void Camera::getHWDebounceValue(qint32 errorCode)
{
    qint32 debounceValueTmp;
    if ((errorCode = is_TriggerDebounce(hCam, TRIGGER_DEBOUNCE_CMD_GET_DELAY_TIME, &debounceValueTmp, sizeof(qint32))) != IS_SUCCESS)
    {
        QString error = "Не удалось получить длительность дребезжания триггера" + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        options.mutable_hw_params()->set_debounce_value(debounceValueTmp);
    }
}

void Camera::updateHWParameters()
{
    qint32 errorCode = 0;

    getHWExposure(errorCode);

    getHWExposureRange(errorCode);

    getHWPixelClock(errorCode);

    getHWPixelClockRange(errorCode);

    getHWFrameTimeRange(errorCode);

    getHWFrameTime(errorCode);

    getHWTriggerDebounce(errorCode);

    getHWDebounceValue(errorCode);

}

void Camera::fillCurrentCameraParameters()
{
    updateHWParameters();
}



bool Camera::camSeqBuild()
{
    bool bRet = false;
    qint32 nRet;

    //    double FrameTimeMin, FrameTimeMax, FrameTimeIntervall;
    //    nRet = is_GetFrameTimeRange (hCam, &FrameTimeMin, &FrameTimeMax, &FrameTimeIntervall);
    //    if (nRet == IS_SUCCESS)
    //    {
    //        double maxBuffers;
    //        maxBuffers = (1.0f / FrameTimeMin) + 0.5f;
    //        m_nNumberOfBuffers = (qint32) (maxBuffers);
    //        //qDebug() << "BUFFERS " << maxBuffers;
    //        if( m_nNumberOfBuffers < 3 )
    //        {
    //            m_nNumberOfBuffers = 3;
    //        }

    //    }
    //    else
    //        return false;


    // calculate the image buffer width and height , watch if an (absolute) AOI is used
    IS_SIZE_2D imageSize;
    is_AOI(hCam, IS_AOI_IMAGE_GET_SIZE, (void*)&imageSize, sizeof(imageSize));

    options.mutable_hw_params()->set_width(imageSize.s32Width);
    options.mutable_hw_params()->set_height(imageSize.s32Height);

    // allocate buffers (memory) in a loop
    qint32 i;
    for (i = 0; i < maxBufferSize; ++i)
    {
        qint32 iImgMemID = 0;
        char* pcImgMem = 0;

        // allocate a single buffer memory
        nRet = is_AllocImageMem(	hCam,
                                    options.hw_params().width(),
                                    options.hw_params().height(),
                                    bitCount,
                                    &pcImgMem,
                                    &iImgMemID);
        if( nRet != IS_SUCCESS )
        {
            qDebug() << "can't alloc image mem" << nRet;
            break;  // it makes no sense to continue
        }

        // put memory into the sequence buffer management
        nRet = is_AddToSequence(hCam, pcImgMem, iImgMemID);
        if( nRet != IS_SUCCESS )
        {
            qDebug() << "can't add buffer to sequence" << nRet;
            // free latest buffer
            is_FreeImageMem( hCam, pcImgMem, iImgMemID );
            break;  // it makes no sense to continue
        }

        m_viSeqMemId.push_back(iImgMemID);
        m_vpcSeqImgMem.push_back(pcImgMem);

    }

    // enable the image queue
    nRet = is_InitImageQueue (hCam, 0);
    if( nRet == IS_SUCCESS )
    {
        bRet = true;
    }
    else
    {
        qDebug() << "can't init image queue" << nRet;
    }

    return bRet;
}

bool Camera::camSeqKill()
{

    is_ExitImageQueue(hCam);
    is_ClearSequence(hCam);


    for(qint32 i = (maxBufferSize - 1); i >= 0; --i)
    {
        // free buffers
        if(is_FreeImageMem(hCam, m_vpcSeqImgMem.at(i), m_viSeqMemId.at(i)) != IS_SUCCESS )
        {
            return false;
        }
    }

    // no valid buffers any more
    m_viSeqMemId.clear();
    m_vpcSeqImgMem.clear();

    return true;
}

void Camera::doAutoExposure()
{
#ifdef AUTOEXP_MAIN_THREAD
    QSharedPointer <QMetaObject::Connection> conn (new QMetaObject::Connection);
    *conn = connect(&autoExpTimer, &QTimer::timeout, this, [this, conn]()
    {
        if (!streamIsActive)
        {
            autoExpTimer.stop();
            disconnect(*conn);
        }
        if (options.auto_exposure_enable())
        {
            QMutexLocker lockOpt(&recMutex);
            if (!bufferFrames.isEmpty() && bufferFrames.last().mainFrame)
            {
                auto rawFrame = bufferFrames.last().frame;
                lockOpt.unlock();
                Mat frame;
                resize(rawFrame, frame, Size(), 0.5, 0.5);
                frame = frame(cv::Rect(50, 50, frame.cols - 50, frame.rows - 50));

                Mat tmpForAutoExp;
                cvtColor(frame, tmpForAutoExp, CV_BayerBG2GRAY);

                if (autoExpHandler.correct(tmpForAutoExp))
                {

                    QMutexLocker lockOpt(&optionMutex);
                    setExposure(autoExpHandler.getParameters()->exposure());
                    setGain(autoExpHandler.getParameters()->gain());
                    lockOpt.unlock();

                    msg::CameraOptions opt;
                    opt.mutable_hw_params()->set_exposure(options.hw_params().exposure());
                    opt.mutable_hw_params()->set_gain(options.hw_params().gain());
                    opt.set_id(options.id());
                    opt.set_main_add_mode(options.main_add_mode());
                    emit parametersChanged(opt);
                }
            }
        }
    });
    autoExpTimer.setInterval(100);
    autoExpTimer.start();
#else
    cv::setNumThreads(0);
    while (streamIsActive)
       {
           if (options.auto_exposure_enable())
           {
               QMutexLocker lockOpt(&recMutex);
               if (!bufferFrames.isEmpty() && bufferFrames.last().mainFrame)
               {
                   auto rawFrame = bufferFrames.last().frame;
                   QElapsedTimer t, t1, t2, t3, t4;
                   t.start();
                   lockOpt.unlock();
                   Mat frame;
                   t1.start();
                   resize(rawFrame, frame, Size(), 0.5, 0.5);


                   frame = frame(cv::Rect(50, 50, frame.cols - 50, frame.rows - 50));
                   Mat tmpForAutoExp;
                   cvtColor(frame, tmpForAutoExp, CV_BayerBG2GRAY);
                   //int tt2 = t2.elapsed();
                   t3.start();
                   int tt3 = 0;
                   if (autoExpHandler.correct(tmpForAutoExp))
                   {
                       tt3 = t3.elapsed();
                       t4.start();
                       QMutexLocker lockOpt(&optionMutex);
                       t2.start();
                       setExposure(autoExpHandler.getParameters()->exposure());
                       setGain(autoExpHandler.getParameters()->gain());
                       qDebug() << autoExpHandler.getParameters()->gain() << autoExpHandler.getParameters()->exposure() << t1.elapsed() << t2.elapsed()
                                << options.mutable_hw_params()->min_exposure()  << options.mutable_hw_params()->max_exposure();
                       lockOpt.unlock();
                       msg::CameraOptions opt;
                       opt.mutable_hw_params()->set_exposure(options.hw_params().exposure());
                       opt.mutable_hw_params()->set_gain(options.hw_params().gain());
                       opt.set_id(options.id());
                       opt.set_main_add_mode(options.main_add_mode());
                       emit parametersChanged(opt);
                       QThread::msleep(100);
                   }
                   else
                   {
                       QThread::msleep(100);
                   }
               }
           }
    }
#endif


}

void Camera::initializeCameraInterface()
{

    qint32 nRet = is_InitCamera (&hCam, NULL);
    if (nRet == IS_SUCCESS){
        qDebug() << "Camera initialized!";
    }

    qint32 colorMode = format; //IS_CM_SENSOR_RAW8;
    nRet = is_SetColorMode(hCam, colorMode);

    if (nRet == IS_SUCCESS){
        qDebug() << "Camera color mode succesfully set!";
    }

    qint32 displayMode = IS_SET_DM_DIB;
    nRet = is_SetDisplayMode (hCam, displayMode);

    if (nRet == IS_SUCCESS){
        qDebug() << "Display mode is set";
    }

    IS_SIZE_2D sz;
    sz.s32Height = options.hw_params().height();
    sz.s32Width = options.hw_params().width();
    setROI(sz);
    if (options.p_params().rotate())
    {
        is_SetRopEffect(hCam, IS_SET_ROP_MIRROR_UPDOWN | IS_SET_ROP_MIRROR_LEFTRIGHT, 1, 0);
    }

    startLiveVideo();


}



void Camera::setAutoExposure(bool ok)
{
    options.set_auto_exposure_enable(ok);
}

void Camera::setGain(double gain)
{
    options.mutable_hw_params()->set_gain(gain);
    qint32 nRet;
    if (is_SetGainBoost(hCam, IS_GET_GAINBOOST) != IS_SET_GAINBOOST_ON)
    {
        if (is_SetGainBoost(hCam, IS_SET_GAINBOOST_ON) != IS_SET_GAINBOOST_ON)
        {
            QString error = "Не удалось включить gain boost";
            qDebug() << error;
            emit messageFromCameraReady(error);
        }
    }
    if ((nRet = is_SetHardwareGain(hCam, gain, IS_IGNORE_PARAMETER , IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER)) != IS_SUCCESS)
    {
        QString error = "Не удалось изменить усиление " + QString::number(nRet);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
}

void Camera::setDebounceEnable(bool enable)
{
    qint32 nRet;
    if (enable)
    {
        qint32 mode = TRIGGER_DEBOUNCE_MODE_AUTOMATIC;
        if ((nRet = is_TriggerDebounce(hCam, TRIGGER_DEBOUNCE_CMD_SET_MODE, (void*)&mode, sizeof(mode))) == IS_SUCCESS)
        {
            if (options.hw_params().has_debounce_value())
            {
                qint32 value = options.hw_params().debounce_value();
                if ((nRet = is_TriggerDebounce(hCam, TRIGGER_DEBOUNCE_CMD_SET_DELAY_TIME, (void*)&value, sizeof(qint32))) != IS_SUCCESS)
                {
                    QString error = "Не удалось установить величину проверки дребезжания, код ошибки: " + QString::number(nRet);
                    qDebug() << error;
                    emit messageFromCameraReady(error);
                }
            }

        }
        else
        {
            QString error = "Не удалось установить проверку дребезжания: " + QString::number(nRet);
            qDebug() << error;
            emit messageFromCameraReady(error);
        }

    }
    else
    {
        qint32 mode = TRIGGER_DEBOUNCE_MODE_NONE;
        if ((nRet = is_TriggerDebounce(hCam, TRIGGER_DEBOUNCE_CMD_SET_MODE, (void*)&mode, sizeof(qint32))) != IS_SUCCESS)
        {
            QString error = "Не удалось отключить проверку дребезжания, код ошибки: " + QString::number(nRet);
            qDebug() << error;
            emit messageFromCameraReady(error);
        }
    }
}

void Camera::setDebounceValue(qint32 value)
{
    is_TriggerDebounce(hCam, TRIGGER_DEBOUNCE_CMD_SET_DELAY_TIME, (void*)&value, sizeof(qint32));
}

QLinkedList<FrameInfo>::iterator Camera::getLastFrame(bool& ok, bool main)
{
    QMutexLocker lock(&recMutex);
    QLinkedList<FrameInfo>::iterator it;
    if (bufferFrames.begin() == bufferFrames.end())
    {
        qDebug() << "NO FRAMES";
        return bufferFrames.begin();
    }

    else if (main)
    {
        it = bufferFrames.end();
        --it;
        while (!it->mainFrame && it != bufferFrames.begin())
        {
            --it;
        }
        if (it->mainFrame)
        {
            qDebug() << it->time << it->computerTime << "first frame";
            ok = true;
            return it;
        }
    }
    else
    {
        it = bufferFrames.end();
        --it;
        while (it->mainFrame && it != bufferFrames.begin())
        {
            --it;
        }
        if (!it->mainFrame)
        {
            qDebug() << it->time << it->computerTime << "first frame";
            ok = true;
            return it;
        }
    }
    return bufferFrames.begin();
}

void Camera::updateFrameState(QLinkedList <FrameInfo>::iterator& it, bool handled, bool sent)
{
    it->handled = handled;
    it->sent = sent;
    if (it->handled && it->sent)
    {
        it = bufferFrames.erase(it);
    }
}

bool Camera::getNextFrame(QLinkedList<FrameInfo>::iterator& it, bool main)
{
    QMutexLocker lock(&recMutex);
    if (bufferFrames.begin() == bufferFrames.end())
    {
        return false;
    }
    auto itTmp = it;
    ++itTmp;
    const qint32 maxCounter = 3;
    qint32 counter = 0;
    bool waitForNew = false;
    while ((waitForNew = (itTmp == bufferFrames.end()))
            || ((main && !itTmp->mainFrame) || (!main && itTmp->mainFrame))
           && counter != maxCounter)
    {
        lock.unlock();
        QThread::msleep(1000 / options.hw_params().frame_rate());
        lock.relock();
        if (waitForNew)
        {
            itTmp = it;
        }
        ++itTmp;
        ++counter;
    }


        auto last = bufferFrames.last();
       auto test = itTmp;
    //qDebug() << (last.time - test->time) / 10000000.0 << test->time << it->time << last.time;
// qDebug()<< test->time << "delay" << (last.time - test->time) / 10000000.0 << QTime::fromMSecsSinceStartOfDay(test->computerTime);
    it = itTmp;
    return true;
}



void Camera::setBallRecognizeFlag(qint32 value)
{

    streamIsActive = 0;
    QThread::msleep(100);
    if (value)
    {
        emit messageFromCameraReady("Включаю режим распознавания...");
        startRecognition();
    }
    else
    {
        emit messageFromCameraReady("Выхожу из режима распознавания.");
        options.set_ball_recognize_enable(false);
    }

}

void Camera::setBallRecognizeFlagDebug(bool value)
{
    options.set_debug_enable(value);
    recognizer.setDebug(value);
}


void Camera::activateObjectiveController(const QString& pattern, qint32 port)
{
    controller.connectToController(pattern, port);
}


Camera::~Camera()
{
    qDebug() << "camera destroy";
    if (streamIsActive)
    {
        streamIsActive = 0;
        stopLiveVideo();
    }
    is_ExitCamera(hCam);
    saveSettings();
}

void Camera::loadSettings()
{
    QSettings settings;
    QByteArray array;
    array = settings.value("params", QByteArray()).toByteArray();
    if (!array.isEmpty())
    {
        options.ParseFromArray(array, array.size());
    }
}

void Camera::saveSettings()
{
    QSettings settings;
    qint32 size = options.ByteSize();
    QByteArray camParams(options.ByteSize(), '\0');
    options.SerializeToArray(camParams.data(), size);
    settings.setValue("params", camParams);
    settings.sync();
}

//void Camera::debugPedestrianDraw(const QString &path)
//{
//    CalibrationHelper& calibHelper = CalibrationHelper::instance();
//    calibHelper.setCurrentCamera("4510");
//    Mat imgMax = imread("calibrate/MAX-ZOOM/MAX-CENTER.bmp");
//    Mat imgMed = imread("calibrate/MED-ZOOM/MED-CENTER.bmp");
//    Mat imgMin = imread("calibrate/MIN-ZOOM/MIN-CENTER.bmp");
//    Calibration::ExteriorOr eOr3850, eOr4510, eOrMin, eOrMed, eOrMax;
//    Calibration::SpacecraftPlatform::CAMERA::CameraParams cam3850, cam4510, camMin, camMed, camMax;

//    calibHelper.readCurrentCalibrationParameters("3850", "", eOr3850, cam3850, false);
//    calibHelper.setEO("3850", eOr3850);
//    calibHelper.setCameraParams("3850", cam3850);

//    calibHelper.readCurrentCalibrationParameters("4510", "", eOr4510, cam4510, false);
//    calibHelper.setEO("4510", eOr4510);
//    calibHelper.setCameraParams("4510", cam4510);

//    calibHelper.readCurrentCalibrationParameters("MAX-CENTER", "", eOrMax, camMax, false);
//    calibHelper.setEO("MAX-CENTER", eOrMax);
//    calibHelper.setCameraParams("MAX-CENTER", camMax);

//    calibHelper.readCurrentCalibrationParameters("MED-CENTER", "", eOrMed, camMed, false);
//    calibHelper.setEO("MED-CENTER", eOrMed);
//    calibHelper.setCameraParams("MED-CENTER", camMed);
//    calibHelper.readCurrentCalibrationParameters("MIN-CENTER", "", eOrMin, camMin, false);
//    calibHelper.setEO("MIN-CENTER", eOrMin);
//    calibHelper.setCameraParams("MIN-CENTER", camMin);

//    VideoCapture cap(path.toStdString());
//    qint32 length = cap.get(cv::CAP_PROP_FRAME_COUNT);
//    Mat frame;
//    streamIsActive = 1;

//    QString timeFilePath = path;

//    QFile f (timeFilePath.replace("avi", "txt"));
//    qDebug() << f.open(QIODevice::ReadOnly);
//    QTextStream in(&f);


//    pedTracker.clear();
//    qint32 skipPedCounter = 0;

//    bufferFrames.clear();
//    pedTracker.setDebugBuffer(&bufferFrames);

//    for (qint32 i = 0; i < length; i++)
//    {
//        if (bufferFrames.size() > 200)
//        {
//            qint32 i = 0;
//            while (i < 100)
//            {
//                bufferFrames.removeFirst();
//                ++i;
//            }
//        }
//        cap >> frame;
//        qint64 time = in.readLine().split("\t").first().toLongLong();
//        FrameInfo ft;
//        frame.copyTo(ft.frame);
//        ft.time = time;
//        bufferFrames.append(FrameInfo(frame, time));
//        if (frame.empty())
//            return;

//        if (!skipPedCounter)
//        {
//            pedTracker.track(frame, time);
//            pedTracker.drawROIs(frame);
//            imshow("window2", frame);
//            Mat draw;
//            imgMed.copyTo(draw);
//            for (auto& i : pedTracker.getPlayersInfo())
//            {
//                if (i->moves.size() > 2)
//                {
//                    for (qint32 j = 1; j < i->moves.size(); ++j)
//                    {
//                        Calibration::Position2D p1, p2;
//                        Calibration::Position p3d1, p3d2;
//                        p3d1.X = i->moves[j - 1].second.onSpace.x;
//                        p3d1.Y = i->moves[j - 1].second.onSpace.y;
//                        p3d1.Z = i->moves[j - 1].second.onSpace.z;
//                        p3d2.X = i->moves[j].second.onSpace.x;
//                        p3d2.Y = i->moves[j].second.onSpace.y;
//                        p3d2.Z = i->moves[j].second.onSpace.z;
//                        Calibration::GetXYfromXYZ(calibHelper.getEO("MED-CENTER"), calibHelper.getCameraParams("MED-CENTER"), p3d1, p1);
//                        Calibration::GetXYfromXYZ(calibHelper.getEO("MED-CENTER"), calibHelper.getCameraParams("MED-CENTER"), p3d2, p2);
//                        line(draw, Point2f(p1.X, p1.Y), Point2f(p2.X, p2.Y), i->color, 5);
//                    }
//                }
//            }
//            imshow("video3", draw);
//            waitKey(0);
//            ++skipPedCounter;
//        }
//        else
//        {
//            skipPedCounter = 0;
//        }

//    }
//}




