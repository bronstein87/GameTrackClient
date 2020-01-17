#include "camera.h"

Camera::Camera(const QString& pattern, bool init, qint32 port, QObject* parent) :
    QObject(parent), controller(pattern, port)
{
    loadSettings();

    if (init)
    {
        initializeCameraInterface();
        assignCameraOptions(options);
        fillCurrentCameraParameters();
        autoExpHandler.params = options.autoExpParams;
        recognizer.setROIs(options.recRois);
        recognizer.setRecognizeParams(options.recParams);
        recognizer.setDebug(options.debugRecFlag);
        connect(&autoExpHandler, &AutoExposureHandler::currentStateReady, this, &Camera::messageFromCameraReady);
    }
    connect(&sendVideoTimer, &QTimer::timeout, [this]()
    {
        QMutexLocker lock(&recMutex);
        auto it = videos.begin();
        while (it != videos.end())
        {
            if (it->leftToAppend == 0 && !videoTaken)
            {
                videoTaken = true;
                lock.unlock();
                emit videoReadyToSend();
                break;
            }
            ++it;
        }
    });

}


void Camera::setTriggerMode(qint32 mode)
{
    stopLiveVideo();
    qint32 errorCode;
    if ((errorCode = is_SetExternalTrigger(hCam, mode)) != IS_SUCCESS)
    {
        QString error = "Не удалось установить триггерный режим, код ошибки: " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {

        startLiveVideo();
        qDebug() <<"trigger";
    }
}

void Camera::tryToStartCamera()
{
    if (!streamIsActive)
    {
        QtConcurrent::run(this, &Camera::procImageQueue);
    }
}



void Camera::procImageQueue()
{
    if (streamIsActive)
    {
        return;
    }
    streamIsActive = 1;
    QtConcurrent::run(this, &Camera::doAutoExposure);
    qint32 i = 0;
    qint32 nMemID = 0;
    char* pBuffer = nullptr;
    qint32 nRet;
    do
    {
        QElapsedTimer t;
        t.start();
        nRet = is_WaitForNextImage(hCam, 1000, &pBuffer, &nMemID);
        if (nRet == IS_SUCCESS)
        {
            auto elapsed = t.elapsed();
            UEYEIMAGEINFO imageInfo;
            nRet = is_GetImageInfo( hCam, nMemID, &imageInfo, sizeof(imageInfo));
            if (nRet == IS_SUCCESS)
            {
                emit currentTimeReady(imageInfo.u64TimestampDevice);

                QTime t = QTime(imageInfo.TimestampSystem.wHour,imageInfo.TimestampSystem.wMinute,
                                imageInfo.TimestampSystem.wSecond, imageInfo.TimestampSystem.wMilliseconds);
                static qint64 timePrev = -1;
                qint64 diff = abs(timePrev - (qint64)imageInfo.u64TimestampDevice);
                qint64 thres = (1. / options.frameRate) * 1e7 + 1e4;

                if (timePrev != -1 && diff > thres)
                {
                    emit this->messageFromCameraReady(QString("Задержка приема кадра %1 (%2)")
                                                      .arg(diff)
                                                      .arg(elapsed));
                }
                timePrev = imageInfo.u64TimestampDevice;
                FrameTime ft;
                ft.time = imageInfo.u64TimestampDevice;
                ft.frame = Mat(options.AOIHeight, options.AOIWidth, CV_8UC1);
                memcpy(ft.frame.data, pBuffer, options.AOIHeight * options.AOIWidth);
                if (options.rotate)
                {
                    Mat matFlip;
                    flip(ft.frame, matFlip, -1);
                    ft.frame = matFlip;
                }
                QMutexLocker lock(&recMutex);
                bufferFrames.append(ft);
                if (bufferFrames.size() >= maxBufferSize)
                {
                    bufferFrames.removeFirst();
                }
                lock.unlock();

                // if (options.rawFrame == 1)
                //{
                emit frameReady(bufferFrames.last().frame, t, imageInfo.u64TimestampDevice);
                // }
                //                else
                //                {
                //                    cv::Mat mat (options.AOIHeight, options.AOIWidth, CV_8UC3);
                //                    handlePicture(matRaw, mat); // делать в отдельном потоке
                //                    emit frameReady(mat.clone(), t, imageInfo.u64TimestampDevice);

                //                }

                // do not forget to unlock the buffer, when all buffers are locked we cannot receive images any more
                is_UnlockSeqBuf (hCam, nMemID, pBuffer);
                ++i;
            }
        }
    }
    while (streamIsActive);
    bufferFrames.clear();
}


void Camera::startRecognition()
{
    QtConcurrent::run(this, &Camera::recognitionProcInternal);
    QtConcurrent::run(this, &Camera::procImageQueueRec);
}

void Camera::recognitionProcInternal()
{
    if (!options.ballRecognizeFlag)
    {
        options.ballRecognizeFlag = 1;
        qint32 skipCounter = 0;
        Mat frame;
        qint32 blockIndex = -1;
        QLinkedList <FrameTime>::iterator it;
        auto conn = connect(&recognizer, &BallRecognizer::resultsReady, this, [this, &blockIndex, &it](auto state) mutable
        {
            auto res = recognizer.getResults();
            if (state == BallRecognizer::BallThrow)
            {
                VideoToSend vs;
                auto dir = res.corWindowsThrow.arrows[res.corWindowsThrow.arrows.size() / 2];
                auto dReal = dir.second - dir.first;
                auto normDReal = norm(dReal);
                dReal = dReal / normDReal;
                auto dCompare = Point2f(recognizer.getThrowDirX(), recognizer.getThrowDirY());
                //qDebug() << acosm(dReal.dot(dCompare));
                if (BOKZMath::acosm(dReal.dot(dCompare)) >= M_PI / 2)
                {
                    emit this->messageFromCameraReady(QString("BALL RECOGNIZED %1 (WRONG DIRECTION)")
                                                      .arg(bufferFrames.size()));
                    blockIndex = -1;
                    return;
                }


                QMutexLocker lock(&recMutex);
                qint32 i = 0;
                QLinkedList <FrameTime>::iterator itcur = bufferFrames.begin();
                while (i < blockIndex)
                {
                    ++itcur; ++i;
                }
                while (itcur != bufferFrames.end())
                {
                    vs.video.append(*itcur);
                    ++itcur;
                }
                QVector <FrameTime> prev, next;
                itcur = bufferFrames.begin();
                while (itcur != bufferFrames.end())
                {
                    ++itcur;
                    if (itcur->time == res.corWindowsThrow.times.first().first)
                    {
                        auto itprev = itcur;
                        qint32 maxPrevFrames = 30;

                        while (itprev != bufferFrames.begin() && prev.size() < maxPrevFrames)
                        {
                            --itprev;
                            prev.append(*itprev);
                        }
                    }
                    if (itcur->time == res.corWindowsThrow.times.last().first)
                    {
                        auto itnext = itcur;
                        qint32 maxNextFrames = 20;
                        while (itnext != bufferFrames.end() && prev.size() < maxNextFrames)
                        {
                            ++itnext;
                            next.append(*itnext);
                        }
                        break;
                    }
                }
                videos.append(vs);
                blockIndex = -1;


                recognizer.analyzePreviousNextFrames(res.corWindowsThrow, prev, next);
                double array[maxNumberOfMeasures][measureDim];
                qint32 ballCount = 0;
                for (qint32 i = 0; i < res.corWindowsThrow.arrows.size(); ++i)
                {
                    array[i][0] = res.corWindowsThrow.arrows[i].second.x;
                    array[i][1] = res.corWindowsThrow.arrows[i].second.y;
                    array[i][2] = (double)res.corWindowsThrow.times[i].first / 10000000.0;
                    array[i][3] = res.corWindowsThrow.times[i].second;
                    if (res.corWindowsThrow.times[i].second)
                    {
                        ++ballCount;
                    }
                }
                qint32 size = res.corWindowsThrow.times.size();
                while (it->time != res.corWindowsThrow.times[size - 1].first)
                {
                    --it;
                }

                for (qint32 i = res.corWindowsThrow.times.size() - 1; i > 0; --i)
                {
                    if (!res.corWindowsThrow.times[i].second
                            || (res.corWindowsThrow.times[i].second
                                && !res.corWindowsThrow.times[i - 1].second))
                    {
                        --it;
                    }
                    else
                    {
                        break;
                    }
                }
                --it; --it;
                auto itb = it;
                auto itsub = itb;
                --itsub;
                
                Mat fb;
                itb->frame.copyTo(fb);
                for (qint32 i = res.corWindowsThrow.times.size() - 1; i > 0; --i) // new
                {
                    if (itb->time == res.corWindowsThrow.times[i].first)
                    {
                        auto bCenter = res.corWindowsThrow.arrows[i].second;
                        auto rect = Rect(bCenter.x - 15, bCenter.y - 15, 30, 30);
                        itsub->frame(rect).copyTo(fb(rect));
                    }
                }
                cvtColor(fb, fb, CV_BayerBG2GRAY);
                recognizer.setBackGroundFrame(fb);
                lock.unlock();

                emit this->messageFromCameraReady(QString("BALL RECOGNIZED %1 TIMES %2 (%3, %4) ")
                                                  .arg(ballCount)
                                                  .arg(bufferFrames.size())
                                                  .arg(vs.video.size())
                                                  .arg(vs.video.first().time));
                //qDebug() << "INIT VIDEO" << vs.video.size() << vs.video.first().time << bufferFrames.size();
                emit this->ballCoordinatesReady(array, vs.video.size() + vs.leftToAppend, res.corWindowsThrow.arrows.size());
            }
            else if (state == BallRecognizer::BallHit)
            {
                double array[maxNumberOfMeasures][measureDim];
                qint32 ballCount = 0;
                for (qint32 i = 0; i < res.corWindowsHit.arrows.size(); ++i)
                {
                    array[i][0] = res.corWindowsHit.arrows[i].second.x;
                    array[i][1] = res.corWindowsHit.arrows[i].second.y;
                    array[i][2] = (double)res.corWindowsHit.times[i].first / 10000000.0;
                    array[i][3] = res.corWindowsHit.times[i].second;
                    if (res.corWindowsHit.times[i].second)
                    {
                        ++ballCount;
                    }
                }
                blockIndex = -1;
                qDebug() << "HIIIIIIIT" << res.corWindowsHit.arrows.size();
                emit this->messageFromCameraReady(QString("HIT RECOGNIZED %1 TIMES")
                                                  .arg(ballCount));
                emit this->hitCoordinatesReady(array, res.corWindowsHit.arrows.size());
            }
            //send everything
        });


        while (options.ballRecognizeFlag)
        {
            QMutexLocker lock(&recMutex);
            if (!bufferFrames.isEmpty())
            {
                if (it == QLinkedList <FrameTime>::iterator())
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
                if (skipCounter == options.ballRecognizeStep)
                {
                    QElapsedTimer t;
                    t.start();

                    skipCounter = 0;
                    frame = it->frame;

                    Mat cvtMat;
                    recognizer.setDebugMessage("CVT");
                    cvtColor(frame, cvtMat, CV_BayerBG2GRAY);
                    recognizer.setDebugMessage("START");
                    //qDebug() << t.elapsed();
                    BallRecognizer::State state = recognizer.recognize(cvtMat, it->time);
                    qint32 elapsed = t.elapsed();
                    if ( state == BallRecognizer::BallHit
                         || state == BallRecognizer::WaitForHit)
                    {
                        auto r = recognizer.getResults();
                        qint64 dd = it->time -
                                r.corWindowsThrow.times[r.corWindowsThrow.times.size() - 1].first;
                        qDebug() << "TIME" << elapsed << dd;

                    }

                    recognizer.setDebugMessage("OBR OCH");
                    QMutexLocker lock(&recMutex);
                    if (state == BallRecognizer::BallThrow && blockIndex == -1)
                    {
                        QLinkedList <FrameTime>::iterator itDiff = bufferFrames.begin();
                        qint32 diff = 0;
                        while (++itDiff != it)
                        {
                            ++diff;
                        }
                        if (diff > 100) // если больше 100 кадров перед первым распознанным, то берем не с начала
                        {
                            blockIndex = diff - 100;
                        }
                        else
                        {
                            blockIndex = 0;
                        }

                    }
                    qint32 saveCounter = -1;
                    if (blockIndex == -1)
                    {
                        if (state == BallRecognizer::WaitForBall)
                        {
                            saveCounter = 60;
                        }
                        else if (state == BallRecognizer::ProbablyBall)
                        {
                            saveCounter = 120;
                        }
                    }

                    if (saveCounter != -1)
                    {
                        recognizer.setDebugMessage("OCH OCR");
                        qint32 i = 0;
                        auto prevIt = it;
                        while (--prevIt != bufferFrames.begin() && i != saveCounter)
                        {
                            ++i;
                        }
                        if (i == saveCounter)
                        {
                            bufferFrames.erase(bufferFrames.begin(), prevIt);
                        }
                    }

                    if (bufferFrames.size() >= maxBufferSize * 1.2)
                    {
                        bufferFrames.removeFirst();
                    }
                    recognizer.setDebugMessage("ZAK OBR OCH");

                }
            }
        }
        recognizer.setDebugMessage("FIN");
        bufferFrames.clear();
        disconnect(conn);
    }
}

void Camera::procImageQueueRec()
{
    if (streamIsActive)
    {
        return;
    }

    streamIsActive = 1;
    QtConcurrent::run(this, &Camera::doAutoExposure);
    qint32 nMemID = 0;
    char* pBuffer = nullptr;
    qint32 nRet;
    do
    {
        QElapsedTimer t;
        t.start();
        nRet = is_WaitForNextImage(hCam, 1000, &pBuffer, &nMemID);
        if (nRet == IS_SUCCESS)
        {
            auto elapsed = t.elapsed();
            UEYEIMAGEINFO imageInfo;
            nRet = is_GetImageInfo( hCam, nMemID, &imageInfo, sizeof(imageInfo));
            if (nRet == IS_SUCCESS)
            {
                emit currentTimeReady(imageInfo.u64TimestampDevice);

                QTime t = QTime(imageInfo.TimestampSystem.wHour, imageInfo.TimestampSystem.wMinute,
                                imageInfo.TimestampSystem.wSecond, imageInfo.TimestampSystem.wMilliseconds);
                static qint64 timePrev = -1;
                qint64 diff = abs(timePrev - (qint64)imageInfo.u64TimestampDevice);
                if (timePrev != -1 && diff > (1. / options.frameRate) * 1e7 + 1e4)
                {
                    emit this->messageFromCameraReady(QString("Задержка приема кадра %1 (%2)")
                                                      .arg(diff)
                                                      .arg(elapsed));
                }
                timePrev = imageInfo.u64TimestampDevice;

                FrameTime ft;
                ft.time = imageInfo.u64TimestampDevice;
                ft.frame = Mat(options.AOIHeight, options.AOIWidth, CV_8UC1);
                memcpy(ft.frame.data, pBuffer, options.AOIHeight * options.AOIWidth);
                if (options.rotate)
                {
                    Mat matFlip;
                    flip(ft.frame, matFlip, -1);
                    ft.frame = matFlip;
                }

                //cvtColor(matRaw, ft.frame, CV_BayerBG2BGR);
                QMutexLocker lock(&recMutex);
                bufferFrames.append(ft);
                auto it = videos.begin();
                while (it != videos.end())
                {
                    if (it->leftToAppend > 0)
                    {
                        it->video.append(ft);
                        --it->leftToAppend;
                    }
                    ++it;
                }

                if (bufferFrames.size() >= maxBufferSize)
                {
                    bufferFrames.removeFirst();
                }
                lock.unlock();

                emit frameReady(bufferFrames.last().frame, t, imageInfo.u64TimestampDevice);
            }
        }
        is_UnlockSeqBuf (hCam, nMemID, pBuffer);
    }
    while (streamIsActive);
}

void Camera::startLiveVideo()
{
    qint32 errorCode;
    if (!is_CaptureVideo(hCam, IS_GET_LIVE))
    {
        if (camSeqBuild() && (errorCode = is_CaptureVideo(hCam, IS_DONT_WAIT)) != IS_SUCCESS)
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

void Camera::debugRecognizeWithVideoInternal(const QString& path)
{
    VideoCapture cap(path.toStdString());
    qint32 length = cap.get(cv::CAP_PROP_FRAME_COUNT);
    Mat frame;
    streamIsActive = 1;
    while (true)
    {
        cap.set(0, cv::CAP_PROP_POS_FRAMES);
        for (qint32 i = 0; i < length; ++i)
        {
            cap >> frame;
            FrameTime ft;
            ft.frame = frame;
            ft.time = i;
            QMutexLocker lock(&recMutex);
            bufferFrames.append(ft);
            lock.unlock();
            QThread::msleep(9);
        }

    }
}

void Camera::debugRecognizeWithVideo(const QString& path)
{
    QtConcurrent::run(this, &Camera::debugRecognizeWithVideoInternal, path);
    //debugRecognizeWithVideoInternal(c);
}

void Camera::debugRecognizeWithVideoOneThread(const QString &path, qint32 cameraNum)
{
    cameraNumN = cameraNum;
    recognizer.clear();
    qDebug() << path << "///////////////////////////////////////////";
    bufferFrames.clear();
    QLinkedList <FrameTime>::iterator it = bufferFrames.begin();
    qint32 skipCounter = 0;
    qint32 blockIndex = -1;
    if (cameraNum == 3850)
    {

        recognizer.setMainROI(Rect(934, 280, 608, 396));
        recognizer.setFirstROI(Rect(806, 280, 866, 570));
        recognizer.setSecondROI(Rect(7, 56, 1829, 823));
        recognizer.setMainHitROI(Rect(468, 427, 1174, 404));
    }
    else
    {
        recognizer.setMainROI(Rect(400, 290, 868, 612));
        recognizer.setFirstROI(Rect(400, 300, 650, 612));
        recognizer.setSecondROI(Rect(39, 22, 1815, 977));
        recognizer.setMainHitROI(Rect(268, 457, 1240, 433));
    }
    qint32 pos = path.indexOf("video_");
    str = path.mid(pos  + 1, str.size() - pos);
    str = str.remove(".avi");
    bool exit = false;
    if (!conn)
    {

        conn = connect(&recognizer, &BallRecognizer::resultsReady, [this, &blockIndex, &exit, &it](auto state) mutable
        {
            QFile f("/home/nvidia/work_video/res/" + str);
            f.open(QIODevice::Append);
            QTextStream out(&f);
            auto res = recognizer.getResults();
            if (state == BallRecognizer::BallThrow)
            {

                QVector <FrameTime> prev, next;
                QLinkedList <FrameTime>::iterator itcur = bufferFrames.begin();
                while (itcur != bufferFrames.end())
                {
                    ++itcur;
                    if ((*itcur).time == res.corWindowsThrow.times.first().first)
                    {
                        auto itprev = itcur;
                        qint32 maxPrevFrames = 10;

                        while (itprev != bufferFrames.begin() && prev.size() < maxPrevFrames)
                        {
                            --itprev;
                            prev.append(*itprev);
                        }
                    }
                    if ((*itcur).time == res.corWindowsThrow.times.last().first)
                    {
                        auto itnext = itcur;
                        qint32 maxNextFrames = 10;
                        while (itnext != bufferFrames.end() && prev.size() < maxNextFrames)
                        {
                            ++itnext;
                            prev.append(*itnext);
                        }
                        break;
                    }
                }
                recognizer.analyzePreviousNextFrames(res.corWindowsThrow, prev, next);

                out << cameraNumN << "\n";
                for (qint32 i = 0; i < res.corWindowsThrow.arrows.size(); ++i)
                {
                    out << res.corWindowsThrow.arrows[i].second.x << " " <<  res.corWindowsThrow.arrows[i].second.y << " "
                        << res.corWindowsThrow.times[i].first << " " <<res.corWindowsThrow.times[i].second << "\n";
                }

                f.close();
                exit = true;
                qint32 size = res.corWindowsThrow.times.size();
                while (it->time != res.corWindowsThrow.times[size - 1].first)
                {
                    --it;
                }

                for (qint32 i = res.corWindowsThrow.times.size() - 1; i > 0; --i)
                {
                    if (!res.corWindowsThrow.times[i].second)
                    {
                        --it;
                    }
                    else
                    {
                        break;
                    }
                }
                --it;
                auto itb = it;
                Mat fb;
                itb->frame.copyTo(fb);
                // cvtColor(fb, fb, CV_BayerBG2GRAY);
                cvtColor(fb, fb, CV_RGB2GRAY);
                //                Mat resizedFb;
                //                resize(fb, resizedFb, Size(fb.rows / 2, fb.cols / 2));
                //                recognizer.setBackGroundFrame(resizedFb);
                recognizer.setBackGroundFrame(fb);
            }
            else if (state == BallRecognizer::BallHit)
            {
                double array[maxNumberOfMeasures][measureDim];
                qint32 ballCount = 0;
                for (qint32 i = 0; i < res.corWindowsHit.arrows.size(); ++i)
                {
                    array[i][0] = res.corWindowsHit.arrows[i].second.x;
                    array[i][1] = res.corWindowsHit.arrows[i].second.y;
                    array[i][2] = (double)res.corWindowsHit.times[i].first / 10000000.0;
                    array[i][3] = res.corWindowsHit.times[i].second;
                    if (res.corWindowsHit.times[i].second)
                    {
                        ++ballCount;
                    }
                }
            }
            //            auto res = recognizer.getResults();
            //            QMutexLocker lock(&recMutex);
            //            QList <FrameTime> resFrames;
            //          //  for (qint32 i = blockIndex; i < bufferFrames.size(); ++i)
            //           // {
            //           //     resFrames.append(bufferFrames[i]);
            //           //}
            //           // blockIndex = -1;
            //            lock.unlock();
            //            //send everything

        });
    }

    VideoCapture cap(path.toStdString());
    qint32 length = cap.get(cv::CAP_PROP_FRAME_COUNT);
    cap.set(CAP_PROP_POS_FRAMES, 100);
    Mat frame;
    streamIsActive = 1;
    options.ballRecognizeStep = 1;

    //    while (true)
    //    {
    Mat prev, res;
    cap.set(0, cv::CAP_PROP_POS_FRAMES);
    QString timeFilePath = path;

    QFile f (timeFilePath.replace("avi", "txt"));
    qDebug() << f.open(QIODevice::ReadOnly);
    QTextStream in(&f);

    //VideoWriter write(String("/home/nvidia/actual_server/server_debug/calibrate/second.avi"), CV_FOURCC('X','V','I','D'), 60, Size(1920, 1080));
    //qDebug() << write.isOpened();
    recognizer.debugPlotFile(path);
    for (qint32 i = 0; i < length; ++i)
    {

        cap >> frame;
        //imwrite("/home/nvidia/work_video/res/AAAAAAA.png", frame);
        //write.write(frame);
        //QStringList l = in.readLine().split("\t").first();
        //in >> time;
        qint64 time = in.readLine().split("\t").first().toLongLong();
        //        putText( frame,  QString::number((*it).time).toStdString(), Point (100, 100),FONT_HERSHEY_PLAIN, 10, CV_RGB(255,255,255));
        //        imshow("video", frame);
        //        waitKey(0);
        //  if (i > 1)
        //  {
        // cvtColor (frame, frame, CV_BGR2GRAY);
        //cvtColor (prev, prev, CV_BGR2GRAY);
        //cv::absdiff(prev, frame, res);
        //Mat cvtMat;
        //cvtColor(frame, cvtMat, CV_RGB2HSV);
        // putText(res, QString::number(i).toStdString(), Point( 100, 100), FONT_HERSHEY_PLAIN, 10, CV_RGB(255,0, 0));
        // MainROIs r;
        //cv::rectangle(frame, Rect(476, 347, 415, 493), CV_RGB(255,0, 0));
        //qDebug() << res.empty() << res.cols << res.rows << "next";
        //           Rect r = selectROI("video",frame);
        //  qDebug() << r.x << r.y << r.width << r.height;
        // qDebug() << i;
        //imshow("video", frame);
        //waitKey(0);
        //continue;
        //    }
        //   frame.copyTo(prev);

        // continue;
        FrameTime ft;
        frame.copyTo(ft.frame);
        ft.time = time;
        bufferFrames.append(ft);

        //        qDebug() << "TTTT" << time;
        if (!bufferFrames.isEmpty())
        {
            if (++it != bufferFrames.end())
            {
                ++skipCounter;
            }
            else
            {
                --it;
            }

            if (skipCounter == options.ballRecognizeStep)
            {
                skipCounter = 0;
                Mat cvtMat;
                cvtColor((*it).frame, cvtMat, CV_RGB2GRAY);

                QElapsedTimer t;
                t.start();
                BallRecognizer::State state = recognizer.recognize(cvtMat, (*it).time);
                if (state == BallRecognizer::WaitForHit || state == BallRecognizer::BallHit)
                {
                    qDebug() << t.elapsed();
                }
                //                exit = true;
                //                if (exit)
                //                {
                //                    imwrite(QString(path + ".png").toStdString(), cvtMat);
                //                    disconnect(conn);
                //                    return;
                //                }
                if (state == BallRecognizer::BallThrow && blockIndex == -1)
                {
                    QLinkedList <FrameTime>::iterator itDiff = bufferFrames.begin();
                    qint32 diff = 0;
                    while (++itDiff != it)
                    {
                        ++diff;
                    }
                    if (diff > 100)
                    {
                        blockIndex = diff - 100;
                    }
                    else
                    {
                        blockIndex = 0;
                    }

                }
                qint32 saveCounter = -1;
                if (blockIndex == -1)
                {
                    if (state == BallRecognizer::WaitForBall)
                    {
                        saveCounter = 25;
                    }
                    else if (state == BallRecognizer::ProbablyBall)
                    {
                        saveCounter = 50;
                    }
                }
                if (saveCounter != -1)
                {
                    qint32 i = 0;
                    auto prevIt = it;
                    while (--prevIt != bufferFrames.begin() && i != saveCounter)
                    {
                        ++i;
                    }
                    if (i == saveCounter)
                    {
                        bufferFrames.erase(bufferFrames.begin(), prevIt);
                    }
                }

                if (bufferFrames.size() >= maxBufferSize * 1.5)
                {
                    emit messageFromCameraReady("Буфер переполнен!");
                    QMutexLocker lock(&recMutex);
                    bufferFrames.removeFirst();
                    lock.unlock();
                }

            }

        }
    }
    //write.release();

    //}
}




void Camera::debugRecognizeProc()
{
    options.ballRecognizeStep = 2;
    recognitionProcInternal();
    //QtConcurrent::run(this, &Camera::recognitionProcInternal);
}

void Camera::unlockAllBuffer()
{
    qint32 errorCode = -1;
    for (qint32 i = (m_nNumberOfBuffers - 1); i >= 0; --i)
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
    options.focusing = tmp.toInt();
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
    if ((errorCode = is_SetFrameRate(hCam, fps, &options.frameRate))  != IS_SUCCESS)
    {
        QString error = "не удалось установить новый фреймрейт, код ошибки: " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    qDebug() << "NEW FPS1: " << options.frameRate;

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

    if ((errorCode = is_Exposure(hCam, IS_EXPOSURE_CMD_SET_EXPOSURE, (void*)&(exposure), sizeof(exposure))) !=
            IS_SUCCESS)
    {
        QString error = "не удалось установить новую экспозицию, код ошибки: " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        autoExpHandler.params.exposure = exposure;
    }

}

void Camera::assignCameraOptions(const CameraOptions& options)
{
    QMutexLocker lock(&optionMutex);
    // qDebug() << "set options";

    if (options.triggerMode != -1)
    {
        setTriggerMode(options.triggerMode);
    }
    if (options.autoExposureIntenal != -1.0)
    {
        setAutoExposure((bool)options.autoExposureIntenal);
    }
    if (options.pixelClock != -1)
    {
        setPixelClock(options.pixelClock);
        autoExpHandler.params.exposure = currentParameters.exposure;
    }
    if (!qFuzzyCompare(options.exposure, -1.0)
            && options.autoExposureIntenal != 1)
    {
        setExposure(options.exposure);
    }
    if (options.focusing != -1)
    {
        QString focusing = "M" + QString::number(options.focusing) + "#";
        qDebug() << "fc" << focusing;
        setFocusing(focusing);
    }
    if (options.frameRate != -1)
    {
        setFrameRate(options.frameRate);
    }
    if (options.pictureParamFlag != -1)
    {
        setHandlePicture(options.pictureParamFlag);
    }
    if (options.equalization != -1.0)
    {
        setEqualization(options.equalization);
    }
    if (options.gain != -1)
    {
        setGain(options.gain);
    }
    if (!qFuzzyCompare(options.gamma, -1.0))
    {
        setGamma(options.gamma);
    }
    if (!qFuzzyCompare(options.sharp, -1.0))
    {
        setSharpness(options.sharp);
    }
    if (options.whiteBalance != -1)
    {
        setWhiteBalance(options.whiteBalance);
    }
    if (options.IRCorrection != -1)
    {
        setWhiteBalance(options.IRCorrection);
    }

    if (options.saturation != defaultHSV)
    {
        setSaturation(options.saturation);
    }

    if (options.rSaturation != defaultHSV)
    {
        setRSaturation(options.rSaturation);
    }

    if (options.gSaturation != defaultHSV)
    {
        setGSaturation(options.gSaturation);
    }

    if (options.bSaturation != defaultHSV)
    {
        setBSaturation(options.bSaturation);
    }
    if (options.hue != defaultHSV)
    {
        setHue(options.hue);
    }

    if (!qFuzzyCompare(options.shadowCoef, -1.0))
    {
        setShadowCoef(options.shadowCoef);
    }

    if (options.shadowThreshold != -1)
    {
        setShadowThreshold(options.shadowThreshold);
    }

    if (options.shadowGaussWindowSize != -1)
    {
        setShadowWindowSize(options.shadowGaussWindowSize);
    }

    if (!options.wbRect.empty())
    {
        setWBROI(options.wbRect);
    }

    if (!options.recRois.mainSearchRect.empty())
    {
        this->options.recRois.mainSearchRect = options.recRois.mainSearchRect; //tmp
        recognizer.setMainROI(options.recRois.mainSearchRect);
    }

    if (!options.recRois.trackFirstRect.empty())
    {
        this->options.recRois.trackFirstRect = options.recRois.trackFirstRect;
        recognizer.setFirstROI(options.recRois.trackFirstRect);
    }

    if (!options.recRois.trackSecondRect.empty())
    {
        this->options.recRois.trackSecondRect = options.recRois.trackSecondRect;
        recognizer.setSecondROI(options.recRois.trackSecondRect);
    }

    if (!options.recRois.mainHitSearchRect.empty())
    {
        this->options.recRois.mainHitSearchRect = options.recRois.mainHitSearchRect;
        recognizer.setMainHitROI(options.recRois.mainHitSearchRect);
    }

    if (options.autoExpParams.light != AutoExposureHandler::LightningParameter::Invalid)
    {
        autoExpHandler.params.light = options.autoExpParams.light;
    }

    if (!qFuzzyCompare(options.autoExpParams.minGainCoeff, -1))
    {
        autoExpHandler.params.minGainCoeff = options.autoExpParams.minGainCoeff;
    }

    if (!qFuzzyCompare(options.autoExpParams.maxGainCoeff, -1))
    {
        autoExpHandler.params.maxGainCoeff = options.autoExpParams.maxGainCoeff;
    }

    if (!qFuzzyCompare(options.autoExpParams.maxPercent, -1))
    {
        autoExpHandler.params.maxPercent = options.autoExpParams.maxPercent;
    }

    if (!qFuzzyCompare(options.autoExpParams.minRelCoef, -1))
    {
        autoExpHandler.params.minRelCoef= options.autoExpParams.minRelCoef;
    }

    if (!qFuzzyCompare(options.autoExpParams.maxRelCoef, -1))
    {
        autoExpHandler.params.maxRelCoef = options.autoExpParams.maxRelCoef;
    }

    if (options.rawFrame != -1)
    {
        setRawFrame(options.rawFrame);
        //qDebug() << "rawFrameSet";
    }

    if (options.videoDuration != -1)
    {
        setVideoDuration(options.videoDuration);
    }

    if (options.recParams.cannyThresMax != -1)
    {
        recognizer.setMaxCanny(options.recParams.cannyThresMax);
    }

    if (options.recParams.cannyThresMin != -1)
    {
        recognizer.setMinCanny(options.recParams.cannyThresMin);
    }

    if (!qFuzzyCompare(options.recParams.circularityCoeff, -1.0))
    {
        recognizer.setCircularityCoef(options.recParams.circularityCoeff);
    }

    if (!qFuzzyCompare(options.recParams.corrCoef, -1.0))
    {
        recognizer.setCorrCoef(options.recParams.corrCoef);
    }

    if (!qFuzzyCompare(options.recParams.maxAngleBetwDirections, -1.0))
    {
        recognizer.setMaxAngleBetwDirections(options.recParams.maxAngleBetwDirections);
    }

    if (options.recParams.maxArea != -1)
    {
        recognizer.setMaxArea (options.recParams.maxArea );
    }

    if (options.recParams.minArea != -1)
    {
        recognizer.setMinArea (options.recParams.minArea );
    }

    if (options.recParams.searchAreaSize != -1)
    {
        recognizer.setSearchAreaSize( options.recParams.searchAreaSize );
    }

    if (!qFuzzyCompare(options.recParams.minSkoOnTemplate, -1.0))
    {
        recognizer.setMinSkoOnTemplate(options.recParams.minSkoOnTemplate);
    }

    if (!qFuzzyCompare(options.recParams.minSpeed, -1.0))
    {
        recognizer.setMinSpeed(options.recParams.minSpeed);
    }

    if (!qFuzzyCompare(options.recParams.skoCoef, -1.0))
    {
        recognizer.setSkoCoef(options.recParams.skoCoef);
    }

    if (!qFuzzyCompare(options.recParams.dirX, -1))
    {
        recognizer.setThrowDirX(options.recParams.dirX);
    }

    if (!qFuzzyCompare(options.recParams.dirY, -1))
    {
        recognizer.setThrowDirY(options.recParams.dirY);
    }

    if (options.ballRecognizeFlag != -1)
    {
        setBallRecognizeFlag(options.ballRecognizeFlag);
    }

    if (options.ballRecognizeStep != -1)
    {
        setBallRecognizeStep(options.ballRecognizeStep);
    }

    if (options.debugRecFlag != -1)
    {
        setBallRecognizeFlagDebug(options.debugRecFlag);
    }

    if (options.debounceEnable != -1)
    {
        setDebounceEnable(options.debounceEnable);
    }

    if (options.debounceValue != -1)
    {
        setDebounceValue(options.debounceValue);
    }

    if (options.rotate != -1)
    {
        setRotate(options.rotate);
    }


    emit parametersChanged();

}



void Camera::fillCurrentCameraParameters()
{
    qint32 errorCode;
    if ((errorCode = is_Exposure(hCam, IS_EXPOSURE_CMD_GET_EXPOSURE, (void*)&(currentParameters.exposure),
                                 sizeof(currentParameters.exposure))) != IS_SUCCESS)
    {
        QString error = "не удалось получить экспозицию, код ошибки: " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        options.exposure = currentParameters.exposure;
    }
    if ((errorCode = is_Exposure(hCam, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MIN,
                                 (void*)&(currentParameters.minExposure), sizeof(currentParameters.minExposure))) != IS_SUCCESS)
    {
        QString error = "не удалось получить мин. экспозицию, код ошибки: " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    if ((errorCode = is_Exposure(hCam, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MAX,
                                 (void*)&(currentParameters.maxExposure), sizeof(currentParameters.maxExposure))) != IS_SUCCESS)
    {
        QString error = "не удалось получить макс. экспозицию, код ошибки: " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    if ((errorCode = is_PixelClock(hCam, IS_PIXELCLOCK_CMD_GET, (void*)&currentParameters.pixelClock,
                                   sizeof(currentParameters.pixelClock))) != IS_SUCCESS)
    {
        QString error = "не удалось установить новый пиксел клок, код ошибки: " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        options.pixelClock = currentParameters.pixelClock;
    }


    options.triggerMode = currentParameters.triggerMode = is_SetExternalTrigger(hCam, IS_GET_EXTERNALTRIGGER);

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
        currentParameters.minPixelClock = nRange[0];
        currentParameters.maxPixelClock = nRange[1];
    }
    double dummy;
    if ((errorCode = is_GetFrameTimeRange(hCam, &currentParameters.minFrameRate,
                                          &currentParameters.maxFrameRate, &dummy)) != IS_SUCCESS)
    {
        QString error = "Не удалось получить рендж фреймрета " + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        double tmp = currentParameters.maxFrameRate;
        currentParameters.maxFrameRate = 1. / currentParameters.minFrameRate;
        currentParameters.minFrameRate = 1. / tmp;
    }
    if ((errorCode = is_GetFramesPerSecond (hCam, &currentParameters.frameRate)) != IS_SUCCESS)
    {
        QString error = "Не удалось получить текущий фреймрейт" + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        currentParameters.frameRate = std::round(currentParameters.frameRate);
        options.frameRate = currentParameters.frameRate;
    }

    if ((errorCode = is_TriggerDebounce(hCam, TRIGGER_DEBOUNCE_CMD_GET_MODE, &currentParameters.debounceEnable, sizeof(qint32))) != IS_SUCCESS)
    {
        QString error = "Не удалось получить режим дребезжания триггера" + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        options.debounceEnable = currentParameters.debounceEnable;
    }

    if ((errorCode = is_TriggerDebounce(hCam, TRIGGER_DEBOUNCE_CMD_GET_DELAY_TIME, &currentParameters.debounceValue, sizeof(qint32))) != IS_SUCCESS)
    {
        QString error = "Не удалось получить длительность дребезжания триггера" + QString::number(errorCode);
        qDebug() << error;
        emit messageFromCameraReady(error);
    }
    else
    {
        options.debounceValue = currentParameters.debounceValue;
    }

    currentParameters.gain = options.gain;
    currentParameters.autoExpParams = autoExpHandler.params;
    currentParameters.wbRect = options.wbRect;
    currentParameters.recRois = options.recRois;
    currentParameters.rawFrame = options.rawFrame;
    currentParameters.width = options.AOIWidth;
    currentParameters.height = options.AOIHeight;
    currentParameters.videoDuration = options.videoDuration;
    currentParameters.triggerMode = options.triggerMode;
    currentParameters.pictureParamFlag = options.pictureParamFlag;
    currentParameters.whiteBalance = options.whiteBalance;
    currentParameters.equalization = options.equalization;
    currentParameters.autoExposureIntenal = options.autoExposureIntenal;
    currentParameters.focusing = options.focusing;
    currentParameters.gamma = options.gamma;
    currentParameters.sharp = options.sharp;
    currentParameters.portSendStream = options.portSendStream;
    currentParameters.IRCorrection = options.IRCorrection;
    currentParameters.saturation = options.saturation;
    currentParameters.rSaturation = options.rSaturation;
    currentParameters.gSaturation = options.gSaturation;
    currentParameters.bSaturation = options.bSaturation;
    currentParameters.hue = options.hue;
    currentParameters.shadowCoef = options.shadowCoef;
    currentParameters.shadowThreshold = options.shadowThreshold;
    currentParameters.shadowGaussWindowSize = options.shadowGaussWindowSize;
    currentParameters.recParams = recognizer.getRecognizeParameters();
    currentParameters.debugRecFlag = recognizer.getDebug();
    currentParameters.ballRecognizeFlag = options.ballRecognizeFlag;
    currentParameters.ballRecognizeStep = options.ballRecognizeStep;
    currentParameters.rotate = options.rotate;
}



void Camera::handlePicture(Mat& rawframe, Mat& frame)
{
    cvtColor(rawframe, frame, CV_BayerBG2BGR);
    QMutexLocker lock(&optionMutex);
    if (options.pictureParamFlag)
    {
        if (options.whiteBalance == 1)
        {
            Ptr<xphoto::GrayworldWB> wb = xphoto::createGrayworldWB();
            wb->setSaturationThreshold(0.98);
            wb->balanceWhite(frame, frame);
            //qDebug() << "do wb";
        }
        if (options.gain != -1
                && options.gain != 0)
        {
            frame = gain(frame, 1, options.gain);
            //qDebug() << "do gain" << options.gain;
        }

        if (!qFuzzyCompare(options.gamma, -1.0)
                && !qFuzzyCompare(options.gamma, 1.0))
        {
            frame = gammaCorrection(frame, options.gamma); // может делать её предпоследней перед резкостью?
            //qDebug() << "do gamma" << options.gamma;
        }

        if (options.rSaturation != defaultHSV)
        {
            frame = gainChannel(0, frame, 1, options.rSaturation);
            // qDebug() << "do rsat";
        }

        if (options.gSaturation != defaultHSV)
        {
            frame = gainChannel(1, frame, 1, options.gSaturation);
            //qDebug() << "do gsat";
        }

        if (options.bSaturation != defaultHSV)
        {
            frame = gainChannel(2, frame, 1, options.bSaturation);
            //qDebug() << "do bsat";
        }

        cvtColor(frame, frame, CV_RGB2HSV);
        if (options.equalization == 1)
        {
            frame = equalizeHistogramm(frame);
            // qDebug() << "do hist";
        }

        if (options.hue != defaultHSV)
        {
            frame = gainChannel(0, frame, 1, options.hue);
            //qDebug() << "do sat";
        }

        if (options.saturation != defaultHSV)
        {
            frame = gainChannel(1, frame, 1, options.saturation);
            //qDebug() << "do sat" << options.saturation;
        }

        if (!qFuzzyCompare(options.shadowCoef, -1.0)
                && options.shadowThreshold != -1
                && options.shadowGaussWindowSize != -1)
        {
            frame = shadowing(frame, options.shadowThreshold * 255, options.shadowGaussWindowSize,
                              options.shadowCoef);
            //qDebug() << "do shad";
        }

        if (!qFuzzyCompare(options.sharp, -1.0)
                && !qFuzzyCompare(options.sharp, 0))
        {
            frame = unsharpMasking(frame, options.sharp);
            //qDebug() << "do sharp";
        }
        lock.unlock();
        cvtColor(frame, frame, CV_HSV2RGB);
    }
}


bool Camera::camSeqBuild()
{
    bool bRet = false;
    qint32 nRet;

    double FrameTimeMin, FrameTimeMax, FrameTimeIntervall;
    nRet = is_GetFrameTimeRange (hCam, &FrameTimeMin, &FrameTimeMax, &FrameTimeIntervall);
    if (nRet == IS_SUCCESS)
    {
        double maxBuffers;
        maxBuffers= (1.0f / FrameTimeMin) + 0.5f;
        m_nNumberOfBuffers = (qint32) (maxBuffers);
        //qDebug() << "BUFFERS " << maxBuffers;
        if( m_nNumberOfBuffers < 3 )
        {
            m_nNumberOfBuffers = 3;
        }

    }
    else
        return false;


    // calculate the image buffer width and height , watch if an (absolute) AOI is used
    IS_SIZE_2D imageSize;
    is_AOI(hCam, IS_AOI_IMAGE_GET_SIZE, (void*)&imageSize, sizeof(imageSize));

    options.AOIWidth = imageSize.s32Width;
    options.AOIHeight = imageSize.s32Height;


    // allocate buffers (memory) in a loop
    qint32 i;
    for (i = 0; i < m_nNumberOfBuffers; ++i)
    {
        qint32 iImgMemID = 0;
        char* pcImgMem = 0;

        // allocate a single buffer memory
        nRet = is_AllocImageMem(	hCam,
                                    options.AOIWidth,
                                    options.AOIHeight,
                                    bitCount,
                                    &pcImgMem,
                                    &iImgMemID);
        if( nRet != IS_SUCCESS )
        {
            break;  // it makes no sense to continue
        }

        // put memory into the sequence buffer management
        nRet = is_AddToSequence(	hCam, pcImgMem, iImgMemID);
        if( nRet != IS_SUCCESS )
        {
            // free latest buffer
            is_FreeImageMem( hCam, pcImgMem, iImgMemID );
            break;  // it makes no sense to continue
        }

        m_viSeqMemId.push_back(iImgMemID);
        m_vpcSeqImgMem.push_back(pcImgMem);

    }

    // store current number buffers in case we did not match to get the desired number
    m_nNumberOfBuffers = i;

    // enable the image queue
    nRet = is_InitImageQueue (hCam, 0);
    if( nRet == IS_SUCCESS )
    {
        // we got buffers in the image queue
        if( m_nNumberOfBuffers >= 3)
            bRet= true;
    }


    return bRet;
}

bool Camera::camSeqKill()
{

    is_ExitImageQueue(hCam);
    is_ClearSequence(hCam);


    for(qint32 i = (m_nNumberOfBuffers-1); i >= 0   ; --i)
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
    m_nNumberOfBuffers = 0;

    return true;
}

void Camera::doAutoExposure()
{
    while (streamIsActive)
    {
        if (options.autoExposureIntenal == 1)
        {
            QMutexLocker lockOpt(&recMutex);
            if (!bufferFrames.isEmpty())
            {
                auto rawFrame = bufferFrames.last().frame;
                lockOpt.unlock();
                Mat frame;
                cvtColor(rawFrame, frame, CV_BayerBG2RGB);
                Mat tmpForAutoExp;
                cvtColor(frame, tmpForAutoExp, CV_RGB2HSV);
                if (autoExpHandler.correct(tmpForAutoExp))
                {
                    QMutexLocker lockOpt(&optionMutex);
                    setExposure(autoExpHandler.params.exposure);
                    setGain(autoExpHandler.params.gain);
                    lockOpt.unlock();
                    emit parametersChanged();

                }
            }

        }
    }

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

    IS_SIZE_2D sz;
    sz.s32Height = options.AOIHeight;
    sz.s32Width = options.AOIWidth;
    setROI(sz);
    startLiveVideo();

}



void Camera::setAutoExposure(bool ok)
{
    if (ok)
    {
        if (options.autoExposureIntenal == 0)
        {
            options.autoExposureIntenal = 1;
            setGain(autoExpHandler.params.minGainCoeff);
        }
    }
    else
    {
        options.autoExposureIntenal = 0;
    }
}

void Camera::setGain(double gain)
{
    options.gain = gain;
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

void Camera::setDebounceEnable(qint32 value)
{
    qint32 nRet;
    if (value > 0)
    {
        qint32 mode = TRIGGER_DEBOUNCE_MODE_AUTOMATIC;
        if ((nRet = is_TriggerDebounce(hCam, TRIGGER_DEBOUNCE_CMD_SET_MODE, (void*)&mode, sizeof(mode))) == IS_SUCCESS)
        {
            if ((nRet = is_TriggerDebounce(hCam, TRIGGER_DEBOUNCE_CMD_SET_DELAY_TIME, (void*)&options.debounceValue, sizeof(qint32))) != IS_SUCCESS)
            {
                QString error = "Не удалось установить величину проверки дребезжания, код ошибки: " + QString::number(nRet);
                qDebug() << error;
                emit messageFromCameraReady(error);
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

QLinkedList<FrameTime>::iterator Camera::getLastFrame(bool& ok)
{
    QMutexLocker lock(&recMutex);
    if (bufferFrames.begin() == bufferFrames.end())
    {
        return bufferFrames.begin();
    }
    ok = true;
    return --bufferFrames.end();
}

bool Camera::getNextFrame(QLinkedList<FrameTime>::iterator& it)
{
    QMutexLocker lock(&recMutex);
    if (bufferFrames.begin() == bufferFrames.end())
    {
        return false;
    }
    auto itTmp = it;
    if (++itTmp == bufferFrames.end())
    {
        lock.unlock();
        QThread::msleep(10000 / currentParameters.frameRate);
        itTmp = it;
        if (++itTmp == bufferFrames.end())
        {
            return false;
        }
    }
    auto last = bufferFrames.last();
    auto test = itTmp;
    ++test;
    qDebug() << (last.time - itTmp->time) / 10000000.0 << test->time << it->time << last.time << itTmp->time;
    it = itTmp;
    return true;
}

bool Camera::getReadyVideo(QLinkedList <FrameTime>& video)
{
    QMutexLocker lock(&recMutex);
    if (videoTaken)
    {
        video = videos.first().video;
        qDebug() << "SENNDDDDDDDDDD" << video.first().time << videos.size() << QTime::currentTime();
        emit messageFromCameraReady(QString("Send repeat video - %1 frames (%2)")
                                    .arg(video.size())
                                    .arg(video.first().time));
        videos.removeFirst();
        videoTaken = false;
        return true;
    }
    return false;
}

void Camera::handleCantTakeVideo()
{
    videoTaken = false;
}


void Camera::activateObjectiveController(const QString& pattern, qint32 port)
{
    controller.connectToController(pattern, port);
}


Camera::~Camera()
{
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
    array = settings.value("cam_params", QByteArray()).toByteArray();
    if (!array.isEmpty())
    {
        memcpy(&options, array.data(), array.size());
    }
}

void Camera::saveSettings()
{
    QSettings settings;
    QByteArray camParams;
    options.autoExpParams = autoExpHandler.params;
    options.recParams = recognizer.getRecognizeParameters();
    camParams.append((const char*)&options, sizeof(options));
    settings.setValue("cam_params", camParams);
    settings.sync();
}




