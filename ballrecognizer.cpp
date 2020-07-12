#include "ballrecognizer.h"


BallRecognizer::BallRecognizer(QObject *parent) : QObject(parent)
{
    //cv::namedWindow("video",WINDOW_NORMAL);
    qRegisterMetaType <BallRecognizer::State> ();
}

bool BallRecognizer::setDebug(bool flag)
{
    if (flag)
    {
        QDir dir;
        debugDirName = QString("debug %1").arg(QDate::currentDate().toString("dd_MM_yyyy"));
        dir.mkdir(debugDirName);
        QString path = debugDirName + "/" + QString("debug%1").arg(QTime::currentTime().toString("hh_mm_ss"));
        telemetryFile.setFileName(path);
        bool ok = telemetryFile.open(QIODevice::WriteOnly);
        if (ok)
        {
            debug = 1;
            debugStream.setDevice(&telemetryFile);
            return true;
        }
        else
        {
            debug = 0;
            return false;
        }
    }
    debug = flag;
    return true;
}

void BallRecognizer::analyzePreviousNextFrames(BallRecognizer::CorData& recData, const QVector <FrameTime>& prev, const QVector <FrameTime>& next)
{
    if (!prev.isEmpty())
    {
        Mat pattern = recData.objTemplates.front();
        Point2f from = recData.arrows.first().second;
        Point2f dir = recData.arrows[1].first - recData.arrows[1].second;
        Point2f pos = from + dir;
        Point2f oldCenter = from;
        for (auto & i : prev)
        {
            if (!rois.trackFirstRect.contains(pos))
            {
                break;
            }
            Mat frame;
            cvtColor(i.frame, frame, CV_BayerBG2GRAY);
            //cvtColor(i.frame, frame, CV_BGR2GRAY);
            absdiff(frame, frames.previous, frame);

            qint32 left = pos.x - params.searchAreaSize / 2;
            left = left < 0 ? 0 : left;
            qint32 up = pos.y - params.searchAreaSize / 2;
            up = up < 0 ? 0 : up;
            qint32 h = (up + params.searchAreaSize) > i.frame.rows ?
                        i.frame.rows - up
                      : params.searchAreaSize;
            qint32 w = (left + params.searchAreaSize) > i.frame.cols ?
                        i.frame.cols - left
                      : params.searchAreaSize;
            Rect sRect = Rect(left, up, w, h);
            Mat searchArea = frame(sRect);
            int result_cols =  searchArea.cols - pattern.cols + 1;
            int result_rows = searchArea.rows - pattern.rows + 1;
            Mat result;
            result.create(result_rows, result_cols, CV_32FC1 );
            matchTemplate(searchArea, pattern, result, CV_TM_CCORR_NORMED);
            double minVal; double maxVal; Point minLoc; Point maxLoc;
            Point matchLoc;
            //            rectangle(frame, sRect, CV_RGB(255, 255, 255));

            //                                    imshow("video", searchArea);
            //                                    waitKey(0);
            //                                    imshow("video", pattern);
            //                                    waitKey(0);
            //                                    imshow("video", frame);
            //                                    waitKey(0);
            minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
            matchLoc = maxLoc;
            if (maxVal > params.corrCoef)
            {
                auto center = Point2f ((double)left + ((double)maxLoc.x
                                                       + ((double)pattern.cols / 2)),
                                       (double)up + ((double)maxLoc.y
                                                     + ((double)pattern.rows / 2)));

                auto oldDir = dir;
                auto oldDirNorm = oldDir / norm(oldDir);
                dir = center - oldCenter;
                auto dirNorm = dir / norm(dir);
                //qDebug() << acosm(dirNorm.dot(oldDirNorm)) * radToDegrees << center.x << center.y << maxVal;
                if (acosm(dirNorm.dot(oldDirNorm)) * radToDegrees < params.maxAngleBetwDirections)
                {
                    //                    auto r = Rect(matchLoc.x, matchLoc.y,
                    //                                  pattern.cols,
                    //                                  pattern.rows);
                    //                    searchArea(r).copyTo(pattern);

                    oldCenter = center;
                    recData.arrows.prepend(qMakePair(pos, center));
                    recData.times.prepend(qMakePair(i.time, true));
                }
                else
                {
                    dir = oldDir;
                    oldCenter += dir;
                    recData.arrows.prepend(qMakePair(center, pos));
                    recData.times.prepend(qMakePair(i.time, false));
                }

            }
            else
            {
                oldCenter += dir;
                recData.arrows.prepend(qMakePair(oldCenter, pos));
                recData.times.prepend(qMakePair(i.time, false));
            }

            pos += dir;
        }
    }


    //    for (auto & i : next)
    //    {
    //        imshow("video", i.frame);
    //        waitKey(0);
    //    }
}

void BallRecognizer::debugPlotFile(const QString &filename)
{
    v.clear();
    vinit.clear();
    vold.clear();
    voldinit.clear();
    QString f = filename;
    f = f.remove(".avi");
    qint32 pos = filename.indexOf("T");
    QString time = f.mid(pos + 1);
    QString camera = time.mid(time.indexOf("_") + 1);
    time.remove(camera).remove("_");
    //    plotFile.setFileName(QString("/home/nvidia/work_video/res/plot/%1_%2").arg(camera).arg(time));
    //    if (plotFile.exists())
    //    {
    //        plotFile.open(QIODevice::ReadOnly);
    //        QTextStream in (&plotFile);
    //        QString line;
    //        while (in.readLineInto(&line))
    //        {
    //            auto l = line.split("\t");
    //            v.append(Point2f(l.first().toDouble(), l.last().toDouble()));
    //        }
    //    }
    plotFile.close();
    plotFile.setFileName(QString("/home/nvidia/work_video/res/plot/%1_%2old").arg(camera).arg(time));
    if (plotFile.exists())
    {
        plotFile.open(QIODevice::ReadOnly);
        QTextStream in (&plotFile);
        QString line;
        while (in.readLineInto(&line))
        {
            auto l = line.split("\t");
            vold.append(Point2f(l.first().toDouble(), l.last().toDouble()));
        }
    }


    //    plotFile.close();
    //    plotFile.setFileName(QString("/home/nvidia/work_video/res/plot/%1_%2init").arg(camera).arg(time));
    //    if (plotFile.exists())
    //    {
    //        plotFile.open(QIODevice::ReadOnly);
    //        QTextStream in (&plotFile);
    //        QString line;
    //        while (in.readLineInto(&line))
    //        {
    //            auto l = line.split("\t");
    //            vinit.append(Point2f(l.first().toDouble(), l.last().toDouble()));
    //        }
    //    }


    plotFile.close();
    plotFile.setFileName(QString("/home/nvidia/work_video/res/plot/%1_%2oldinit").arg(camera).arg(time));
    if (plotFile.exists())
    {
        plotFile.open(QIODevice::ReadOnly);
        QTextStream in (&plotFile);
        QString line;
        while (in.readLineInto(&line))
        {
            auto l = line.split("\t");
            voldinit.append(Point2f(l.first().toDouble(), l.last().toDouble()));
        }
    }

    plotFile.close();
    plotFile.setFileName(QString("/home/nvidia/work_video/res/%1").arg(time));
    if (plotFile.exists())
    {
        plotFile.open(QIODevice::ReadOnly);
        QTextStream in (&plotFile);
        QString line;
        bool readData = false;
        while (in.readLineInto(&line))
        {
            if (readData)
            {
                auto l = line.split(" ");
                if (l.size() == 4)
                {
                    vinit.append(Point2f(l.first().toDouble(), l[1].toDouble()));
                }
                else
                {
                    break;
                }
            }
            if (line.toInt() == camera.toInt())
            {
                readData = true;
            }
        }
    }

}

void BallRecognizer::checkNonApriorMeasures(BallRecognizer::CorData& corWin)
{
    if (corWin.dirAccordCount == acceptBallThreshold + 1)
    {
        double meanAngle = 0;
        QList <double> angles;
        angles.prepend(0);
        for (qint32 i = corWin.arrows.size() - 1; i > 1; --i)
        {
            Point2f dirFirst = corWin.arrows[i].second - corWin.arrows[i].first;
            double normFirst = norm(dirFirst);
            dirFirst = dirFirst / normFirst;

            Point2f dirSecond = corWin.arrows[i - 1].second - corWin.arrows[i - 1].first;
            double normSecond = norm(dirSecond);
            dirSecond = dirSecond / normSecond;
            double angle = acosm(dirFirst.dot(dirSecond)) * radToDegrees;
            angles.prepend(angle);
            meanAngle += angle / (corWin.arrows.size() - 1);
        }

        bool remove = false;
        for (qint32 j = 1; j < acceptBallThreshold; ++j)
        {
            if (angles[j] / meanAngle > 2
                    && angles[j] > params.maxAngleBetwDirections / 2)
            {
                angles.removeAt(j);
                corWin.times.remove(j);
                corWin.arrows.remove(j);
                corWin.objTemplates.remove(j);
                corWin.objGrayTemplates.remove(j);
                remove = true;
                --j;
            }
        }
        if (remove)
        {
            corWin.times.removeFirst();
            corWin.arrows.removeFirst();
            corWin.objTemplates.removeFirst();
            corWin.objGrayTemplates.removeFirst();
        }
    }

}

BallRecognizer::State BallRecognizer::recognize(Mat frame, qint64 time)
{
    if (rois.mainSearchRect.empty() ||
            rois.trackFirstRect.empty() ||
            rois.trackSecondRect.empty())
    {
        emit errorOccured("Не заданы ROI.");
    }
//    params.minArea = 30;
//    params.maxArea = 140;
//    params.minSpeed = 15;
//    params.skoCoef = 2.5;
//    params.maxAngleBetwDirections = 13;
//    params.corrCoef = 0.7;
//    params.searchAreaSize = 50;
//    params.cannyThresMax = 150;
//    params.cannyThresMin = 50;
//    params.circularityCoeff = 0.85;

//    if (currentState == WaitForHit || currentState == BallHit)
//    {
//        changeParamsForHit();
//    }

    //    if (currentState == WaitForHit || currentState == BallHit)
    //    {
    //        resize(frame, frames.current, Size(frame.rows / 2, frame.cols / 2));
    //    }
    //    else
    //    {
    //        frames.current = frame;
    //    }

    frames.current = frame;
    if (frames.previous.empty())
    {
        frames.current.copyTo(frames.previous);
    }
    else
    {
        cv::absdiff(frames.current, frames.previous, frames.substracted);
        //if (currentState != WaitForHit && currentState != BallHit)
        //{
        if (previousUpdateCounter == previousUpdateMax)
        {
            previousUpdateCounter = 0;
            frames.background.copyTo(frames.previous);
        }
        else if (previousUpdateCounter == 0)
        {
            frames.current.copyTo(frames.background);
            ++previousUpdateCounter;
        }
        else
        {
            ++previousUpdateCounter;
        }
        //}f

        if (debug)
        {
            tickDt = QDateTime::currentDateTime().toString(Qt::ISODate);
        }

        if (currentState == WaitForBall || currentState == ProbablyBall || currentState == WaitForHit)
        {
            setDebugMessage("NO");
            handleBallNotFound(time);
        }

        setDebugMessage(QString("SLEZH %1 %2 %3 %4 %5 %6")
                        .arg(params.skoCoef).arg(params.corrCoef).arg(params.minSpeed)
                        .arg(params.maxAngleBetwDirections).arg(params.maxArea).arg(params.circularityCoeff));

        handleBallFound(time);


//        imshow("video", frames.current);
//        waitKey(0);

        if (currentState == WaitForHit)
        {
            ++waitForHitCount;
            if (waitForHitCount == maxWaitForHitCount)
            {
                qDebug() << "hit search finished";
                currentState = WaitForBall;
                changeParamsForHit(true);
                waitForHitCount = 0;// unblock queue
                emit resultsReady(currentState);
            }
        }
    }

    setDebugMessage(QString("END %1").arg((qint32)currentState));
    return currentState;
}

void BallRecognizer::setDebugMessage(const QString &str)
{
    if (debug)
    {
        debugStream << QTime::currentTime().toString() << "\t" << str << endl;
        debugStream.flush();
    }
}

void BallRecognizer::removeUnconfirmedOrLost()
{
    qint32 k = 0;

    if (currentState == BallHit || currentState == BallThrow)
    {
        qint32 maxAccord = -1;
        for (qint32 i = 0; i < corWindows.size(); ++i)
        {
            if (maxAccord < corWindows[i].dirAccordCount)
            {
                maxAccord = corWindows[i].dirAccordCount;
            }
        }
        while (k < corWindows.size())
        {
            if (corWindows[k].dirAccordCount < acceptBallThreshold || !corWindows[k].isBall) // такое условие возможно,
                //если мяч был не найден три раза, см handleNotFound
            {
                if (corWindows[k].isBall == true)
                {
                    Q_ASSERT(currentState == WaitForBall); // bred, ostavit tolko isBall
                }
                corWindows.erase(corWindows.begin() + k);
            }
            else if (corWindows[k].isBall && corWindows[k].dirAccordCount < maxAccord)
            {
                corWindows.erase(corWindows.begin() + k);
            }
            else
            {
                ++k;
            }
        }
    }
    else // удаляем все, что было похоже на мяч, но в последствие не было обнаружено
    {
        while (k < corWindows.size())
        {
            if (corWindows[k].notFoundCount >= acceptBallThreshold || corWindows[k].forceDelete)
            {
                corWindows.erase(corWindows.begin() + k);
            }
            else
            {
                ++k;
            }
        }
    }

}

void BallRecognizer::handleBallNotFound(qint64 time)
{
    cv::Scalar m, stdv;
    Rect sRect;
    if (currentState == WaitForHit || currentState == BallHit)
    {
        sRect = rois.mainHitSearchRect;
    }
    else
    {
        sRect = rois.mainSearchRect;
    }
    frames.threshold = frames.substracted(sRect);
    cv::meanStdDev(frames.threshold, m, stdv);

    cv::threshold(frames.threshold, frames.threshold, m[0] + params.skoCoef * stdv[0], 255, cv::THRESH_BINARY);
    cv::Mat subImg = frames.threshold;

    medianBlur(subImg, subImg, 5);
    Mat canny_output;
    vector <vector<Point> > contours;
    Canny(subImg, canny_output, params.cannyThresMin, params.cannyThresMax, 3 );
    findContours( canny_output, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0) );

    if (!contours.empty())
    {
        for (quint32 j = 0; j < contours.size(); ++j)
        {
            auto area = contourArea(contours[j]);
            auto mmnts =  moments( contours[j], true);
            auto circularity = circularityMetric( mmnts );
            auto center_d = Point2f( mmnts.m10 / mmnts.m00 , mmnts.m01 / mmnts.m00 );
            if (area >= params.minArea && area <= params.maxArea
                    && circularity > params.circularityCoeff)
            {
                auto center = Point2f( mmnts.m10 / mmnts.m00 , mmnts.m01 / mmnts.m00 );

                CorData tmp;
                qint32 offset = sqrt(area);
                tmp.center = center;
                tmp.center.x  += sRect.x;
                tmp.center.y  += sRect.y;

                bool dontAdd = false;
                for (const auto& corWin : corWindows)
                {
                    if (abs(corWin.center.x - tmp.center.x) < 30
                            && abs(corWin.center.y - tmp.center.y) < 30)
                    {
                        dontAdd = true;
                    }
                }
                if (dontAdd)
                {
                    continue;
                }

                auto rect = boundingRect(contours[j]);
                rect.x += sRect.x - offset;
                rect.y += sRect.y - offset;
                rect.width += offset  * 2;
                rect.height += offset * 2;
                tmp.rect = rect;

                tmp.objTemplates.push_back(Mat());
                frames.substracted(rect)
                        .copyTo(tmp.objTemplates.back());
                tmp.objGrayTemplates.push_back(Mat());
                frames.current(rect)
                        .copyTo(tmp.objGrayTemplates.back());
                tmp.arrows.append(qMakePair(tmp.center, tmp.center));
                tmp.times.append(qMakePair(time, true));
                corWindows.push_back(tmp);
            }
        }
    }
    if (!corWindows.isEmpty() && currentState != ProbablyBall)
    {
        if (currentState == WaitForBall)
        {
            currentState = ProbablyBall;
        }
        else if (currentState != WaitForHit)
        {
            Q_ASSERT(false);
        }
    }
    else if (currentState != WaitForHit)
    {
        currentState = WaitForBall;
    }
}


void BallRecognizer::handleBallFound(qint64 time)
{
    if (!corWindows.empty())
    {
        for (CorData& corWin : corWindows)
        {
            if (corWin.isNew)
            {
                corWin.isNew = false;
                continue;
            }

            qint32 leftBorder = 0;
            qint32 upBorder = 0;
            qint32 searchArea = 0;
            if (corWin.previousSpeed.x != 0 || corWin.previousSpeed.y != 0)
            {
                if (corWin.notFoundCount > 0)
                {
                    searchArea = params.searchAreaSize / 1.75;
                }
                else
                {
                    searchArea = params.searchAreaSize / 2;
                }

                Point2f p = Point2f(corWin.center.x + corWin.previousSpeed.x,
                                    corWin.center.y + corWin.previousSpeed.y);
                leftBorder = p.x - searchArea;
                upBorder = p.y - searchArea;
            }
            else
            {
                searchArea = params.searchAreaSize;
                leftBorder = corWin.center.x  - params.searchAreaSize;
                upBorder = corWin.center.y  - params.searchAreaSize;
            }

            setDebugMessage("1");

            leftBorder = leftBorder < 0 ?  0 : leftBorder;
            upBorder = upBorder < 0 ?  0 : upBorder;
            auto searchRect = Rect(leftBorder, upBorder,
                                   2 * searchArea, 2 * searchArea);

            searchRect.width = leftBorder + searchRect.width >  frames.substracted.cols
                    ? frames.substracted.cols - leftBorder : searchRect.width;

            searchRect.height = upBorder + searchRect.height >  frames.substracted.rows
                    ? frames.substracted.rows - upBorder : searchRect.height;

            Mat subFlip = frames.substracted(searchRect);

            int result_cols =  subFlip.cols - corWin.objTemplates.back().cols + 1;
            int result_rows = subFlip.rows - corWin.objTemplates.back().rows + 1;

            QString msg = QString("2 %1 %2 %3 %4 %5 %6")
                    .arg(leftBorder).arg(upBorder).arg(searchRect.width)
                    .arg(searchRect.height).arg(result_cols).arg(result_rows);
            setDebugMessage(msg);
            msg = QString("2.1 %1 %2").arg(corWin.center.x).arg(corWin.center.y);
            setDebugMessage(msg);



//            if (currentState == WaitForHit || currentState == BallHit)

//            {

//                //                imshow("video", frames.substracted);

//                //                waitKey(0);

//                imshow("video", subFlip);

//                waitKey(0);

//            }

            if (result_cols <= 0 || result_rows <= 0)
            {
                setDebugMessage("2.!!!!!!");
                corWin.forceDelete = true;
                continue;
            }
            Mat result;
            result.create(result_rows, result_cols, CV_32FC1 );
            matchTemplate(subFlip, corWin.objTemplates.back(), result, CV_TM_CCORR_NORMED);

            double minVal; double maxVal; Point minLoc; Point maxLoc;
            Point matchLoc;

            minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
            matchLoc = maxLoc;

            double corrCoef = (corWin.dirAccordCount > 10 && corWin.notFoundCount == 0)
                    ? params.corrCoef - 0.1 : params.corrCoef;
            setDebugMessage("3");

            if (maxVal < corrCoef && corWin.dirAccordCount > 10)
            {
                matchTemplate(subFlip, corWin.objTemplates[corWin.objTemplates.size() - 2], result, CV_TM_CCORR_NORMED);
                minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
                matchLoc = maxLoc;
            }
            msg = QString("4 %1").arg(maxVal);
            setDebugMessage(msg);
            if (maxVal >= corrCoef)
            {
                Point2f prevCenter = corWin.center;
                qint32 sz =  (params.maxArea - params.minArea) / 2;
                qint32 lb = matchLoc.x - sz;
                lb = lb < 0 ? 0 : lb;
                qint32 ub = matchLoc.y - sz;
                ub = ub < 0 ? 0 : ub;
                qint32 w = lb + corWin.objTemplates.back().cols + 2 * sz;
                w = w > subFlip.cols - 1 ? subFlip.cols - 1 : w;
                qint32 h = ub + corWin.objTemplates.back().rows + 2 * sz;
                h = h > subFlip.rows - 1 ? subFlip.rows - 1 : h;
                auto rectTest = Rect(lb, ub, w - lb, h - ub);
                Mat tmpTemplate;
                subFlip(rectTest).copyTo(tmpTemplate);

                bool takeFromCenter = false;
                bool takeOldDir = false;
                Point2f center;
                cv::Scalar m, stdv;
                setDebugMessage("5");
                if (corWin.isBall /*&& (currentState != WaitForHit && currentState != BallHit)*/)
                {
                    Mat canny_output;
                    vector <vector<Point> > contours;

                    cv::meanStdDev(tmpTemplate, m, stdv);
                    cv::threshold(tmpTemplate, tmpTemplate, m[0] + params.skoCoef * stdv[0], 255, cv::THRESH_BINARY);
                    dilate(tmpTemplate, tmpTemplate, getStructuringElement(MORPH_RECT, Size(3, 3)));

                    Canny(tmpTemplate, canny_output, params.cannyThresMin, params.cannyThresMax, 3);

                    findContours( canny_output, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
                    cv::Moments mmnts;

                    msg = QString("6 %1").arg(contours.size());
                    setDebugMessage(msg);

                    if (contours.size() == 1) // проверяем циркулярность найденного мяча
                    {
                        mmnts = moments( contours.front(), true);
                        double circularity = circularityMetric(mmnts); // хранить среднюю площадь и проверять
                        if (circularity > params.circularityCoeff)
                        {
                            takeFromCenter = true;
                            center = Point2f( mmnts.m10 / mmnts.m00 , mmnts.m01 / mmnts.m00 );
                        }
                    }
                    else
                    {
                        bool anyCircle = false;
                        double prevArea = -1;
                        for (qint32 k = 0; k < contours.size(); ++k)
                        {
                            mmnts = moments(contours[k], true);
                            double circularity = circularityMetric(mmnts); // хранить среднюю площадь и проверять
                            double area = contourArea(contours[k]);

                            if (circularity > params.circularityCoeff
                                    && area > params.minArea)
                            {
                                if (area > prevArea)
                                {
                                    anyCircle = true;
                                    takeFromCenter = true;
                                    center = Point2f( mmnts.m10 / mmnts.m00 , mmnts.m01 / mmnts.m00 );
                                    prevArea = area;
                                }
                            }
                        }
                        if (!anyCircle && corWin.isBall)
                        {
                            msg = QString("6.1 %1 %2").arg(params.circularityCoeff).arg(params.minArea);
                            setDebugMessage(msg);
                            takeOldDir = true;
                        }
                    }
                }


                Point2f testCenter;
                if (!takeFromCenter) // если не очень похоже на мяч берем центр скоррелированной области
                {
                    testCenter = Point2f ((double)leftBorder + ((double)maxLoc.x
                                                                + ((double)corWin.objTemplates.back().cols / 2)) ,
                                          (double)upBorder + ((double)maxLoc.y
                                                              + ((double)corWin.objTemplates.back().rows / 2)));
                }
                else // если очень похоже на мяч, берем центр контура
                {
                    testCenter = Point2f (lb + (double)leftBorder + ((double)center.x),
                                          ub + (double)upBorder + (double)center.y);
                }

                msg = QString("7 %1 %2 %3").arg(takeFromCenter).arg(testCenter.x).arg(testCenter.y);
                setDebugMessage(msg);

                Point2f newDir = testCenter - prevCenter;
                if (corWin.count > 0)
                {
                    double normOld = norm(corWin.dir);
                    Point2f dirOldNorm = corWin.dir / normOld;
                    double normNew = norm(newDir);
                    Point2f dirNewNorm = newDir / normNew;
                    double angle = acosm(dirOldNorm.dot(dirNewNorm)) * radToDegrees;

                    msg = QString("7.1 %1 %2").arg(angle).arg(normNew);
                    setDebugMessage(msg);

                    if (angle < params.maxAngleBetwDirections
                            && (normNew > params.minSpeed || corWin.dirAccordCount >= 5)
                            && !takeOldDir)
                    {
                        ++corWin.dirAccordCount;
                        corWin.notFoundCount = 0;
                    }
                    else if ((angle > params.maxAngleBetwDirections
                              && corWin.dirAccordCount >= acceptBallThreshold) || takeOldDir) // если сильное расхождение
                        // по углу, но мы в зоне броска, возможно скоррелировалось с чем-то другим, поэтому используем старое
                        // направление и по нему считаем примерное положение мяча на следующем кадре
                    {
                        handleNotFound(corWin, time);
                        continue;
                    }
                    else
                    {
                        ++corWin.notFoundCount; // esli eto ne ball i ne proxodit po speed i angle
                        if (corWin.notFoundCount >= acceptBallThreshold && corWin.isBall) // esli ball no po skorosti ne proxodit (krossovok naprimer)
                        {
                            // eto vozmojno nyjno ybrat
                            Q_ASSERT(currentState != WaitForBall && currentState != ProbablyBall);
                            saveResults(corWin);
                            continue;
                        }
                    }
                }
                corWin.center = testCenter;
                corWin.dir = newDir;
                corWin.arrows.append(qMakePair(prevCenter, corWin.center));
                corWin.times.append(qMakePair(time, true));

                setDebugMessage("8");

                if (corWin.dirAccordCount >= acceptBallThreshold)
                {
                    checkNonApriorMeasures(corWin);
                    if (!corWin.previousWasNotFound)
                        corWin.previousSpeed = newDir;
                    corWin.previousWasNotFound = false;

                    if (currentState != BallThrow && currentState != BallHit)
                    {
                        if (currentState == WaitForHit)
                        {
                            currentState = BallHit;
                        }
                        else
                        {
                            currentState = BallThrow;
                        }
                        corWin.isBall = true;
                    }

//                    rectangle(frames.current, Rect(corWin.center.x - 10, corWin.center.y - 10, 20, 20), CV_RGB(255,255,255));
//                    for (auto& ar : corWin.arrows)
//                    {
//                        line(frames.current, ar.first, ar.second, CV_RGB(255, 0, 0));
//                    }
                }

                cv::meanStdDev(corWin.objTemplates.back(), m, stdv);

                msg = QString("9 %1").arg(stdv[0]);
                setDebugMessage(msg);

                if (stdv[0] > params.minSkoOnTemplate)
                {
                    if (takeFromCenter)
                    {
                        Rect r = Rect(lb + center.x - corWin.objTemplates.back().cols / 2,
                                      ub + center.y - corWin.objTemplates.back().rows / 2,
                                      corWin.objTemplates.back().cols + 1,
                                      corWin.objTemplates.back().rows + 1);

                        if (r.x < 0)
                        {
                            r.x = 0;
                        }
                        if (r.y < 0)
                        {
                            r.y = 0;
                        }
                        if (r.x + r.width > subFlip.cols)
                        {
                            r.width = subFlip.cols - r.x;
                        }
                        if (r.y + r.height > subFlip.rows)
                        {
                            r.height = subFlip.rows - r.y;
                        }
                        if (r.width > params.searchAreaSize / 2)
                        {
                            r.width = params.searchAreaSize / 2;
                        }

                        if (r.height > params.searchAreaSize / 2)
                        {
                            r.height = params.searchAreaSize / 2;
                        }


                        corWin.objTemplates.push_back(Mat());
                        subFlip(r).copyTo(corWin.objTemplates.back());
                        r.x = r.x + leftBorder;
                        r.y = r.y + upBorder;
                        corWin.objGrayTemplates.push_back(Mat());
                        frames.current(r).copyTo(corWin.objGrayTemplates.back());
                        setDebugMessage("10.2");
                    }
                    else
                    {
                        auto r = Rect(matchLoc.x, matchLoc.y,
                                      corWin.objTemplates.back().cols,
                                      corWin.objTemplates.back().rows);

                        corWin.objTemplates.push_back(Mat());
                        subFlip(r).copyTo(corWin.objTemplates.back());
                        r.x = r.x + leftBorder;
                        r.y = r.y + upBorder;
                        corWin.objGrayTemplates.push_back(Mat());
                        frames.current(r).copyTo(corWin.objGrayTemplates.back());
                        setDebugMessage("10.1");
                    }
                    checkInSearchArea(corWin);
                    corWin.rect = Rect(Point(leftBorder + matchLoc.x, upBorder + matchLoc.y),
                                       Point(leftBorder + matchLoc.x + corWin.objTemplates.back().cols,
                                             upBorder + matchLoc.y + corWin.objTemplates.back().rows ));
                    setDebugMessage("10.3");
                } //вот тут вопрос что делать? наверное надо добавить хэндл нот фаунд
                checkInSearchArea(corWin);
                ++corWin.count;
            }
            else
            {
                handleNotFound(corWin, time);
            }
        }
        setDebugMessage("10");
        removeUnconfirmedOrLost();
        setDebugMessage("11");
    }
}


double BallRecognizer::circularityMetric(Moments& mmnts)
{
    return pow(mmnts.m00, 2) / (2 * M_PI * (mmnts.mu20 + mmnts.mu02)) ;
}



void BallRecognizer::saveResults(CorData& corWin)
{
    if (currentState == BallThrow)
    {
        results.corWindowsThrow = corWin; // save
        emit resultsReady(currentState);
        corWin.forceDelete = true;
        currentState = WaitForHit;
        changeParamsForHit();
    }
    else if (currentState == BallHit)
    {
        results.corWindowsHit = corWin;// save // unblock queue
        emit resultsReady(currentState);
        corWin.forceDelete = true;
        currentState = WaitForBall;
        changeParamsForHit(true);
    }
}

void BallRecognizer::handleNotFound(CorData& corWin, qint64 time)
{
    if (checkInSearchArea(corWin))
    {
        auto prevCenter = corWin.center;
        corWin.center.x += corWin.dir.x;
        corWin.center.y += corWin.dir.y;
        corWin.arrows.append(qMakePair(prevCenter, corWin.center));
        corWin.times.append(qMakePair(time, false));
        ++corWin.notFoundCount;
        corWin.previousWasNotFound = true;
        if (corWin.notFoundCount >= acceptBallThreshold && corWin.isBall)
        {
            saveResults(corWin);
        }
    }
}


bool BallRecognizer::checkInSearchArea(CorData& corWin)
{
    Point2f prevCenter = corWin.center;
    prevCenter.x += corWin.dir.x;
    prevCenter.y += corWin.dir.y;
    if (currentState == BallThrow)
    {
        if (rois.trackFirstRect.contains(Point2f(prevCenter.x, prevCenter.y)))
        {
            return true;
        }
        else
        {
            if (corWin.isBall)
            {
                results.corWindowsThrow = corWin;// save
                emit resultsReady(currentState);
                corWin.forceDelete = true;
                currentState = WaitForHit;
                changeParamsForHit();
                return false;
            }

        }
    }
    else if (currentState == BallHit)
    {
        if (rois.trackSecondRect.contains(Point2f(prevCenter.x, prevCenter.y)))
        {
            return true;
        }
        else
        {
            if (corWin.isBall)
            {
                results.corWindowsHit = corWin;// save // unblock queue
                emit resultsReady(currentState);
                currentState = WaitForBall;
                corWin.forceDelete = true;
                changeParamsForHit(true);
                return false;
            }
        }
    }
    ++corWin.notFoundCount;
    return false;
}

void BallRecognizer::changeParamsForHit(bool back)
{
    if (back)
    {
        params.circularityCoeff = 0.85;
        params.maxArea = params.maxArea / 2.5;
        params.searchAreaSize = params.searchAreaSize / 2.25;
        params.minSpeed = params.minSpeed + 5;
        waitForHitCount = 0;
    }
    else
    {
        params.circularityCoeff = 0.7;
        params.maxArea = params.maxArea * 2.5;
        params.searchAreaSize = params.searchAreaSize * 2.25;
        params.minSpeed = params.minSpeed - 5;
    }
}


void BallRecognizer::setBackGroundFrame(Mat frame)
{
    previousUpdateCounter = 0;
    frame.copyTo(frames.previous);
}
