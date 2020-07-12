#include "pedestriantracker.h"
#include <QDebug>
#include <QElapsedTimer>

PedestrianTracker::PedestrianTracker(qint32 width, qint32 height, QObject *parent) : DetectNetBase(width, height, parent)
{
    attackInfo.insert(TeamRole::Batter, PPlayer::create());
    attackInfo.insert(TeamRole::FirstBase, PPlayer::create());
    attackInfo.insert(TeamRole::SecondBase, PPlayer::create());
    attackInfo.insert(TeamRole::ThirdBase, PPlayer::create());

    defenceInfo.insert(TeamRole::Pitcher, PPlayer::create());
    defenceInfo.insert(TeamRole::Catcher, PPlayer::create());
    defenceInfo.insert(TeamRole::FirstDefence, PPlayer::create());
    defenceInfo.insert(TeamRole::SecondDefence, PPlayer::create());
    defenceInfo.insert(TeamRole::ThirdDefence, PPlayer::create());
    defenceInfo.insert(TeamRole::ShortStop, PPlayer::create());
    defenceInfo.insert(TeamRole::LeftFielder, PPlayer::create());
    defenceInfo.insert(TeamRole::CenterFielder, PPlayer::create());
    defenceInfo.insert(TeamRole::RightFielder, PPlayer::create());
}


void PedestrianTracker::assignROIs(const qint32 numDetections, detectNet::Detection* detections)
{
    QMutexLocker lock(&mutex);
    for (auto& j : nonStructuredPlayers)
    {
        j->updated = false;
        if (!j->rUse.empty() && j->rDet.empty())
        {
            j->rDet = j->rUse;
        }
    }
    QVector < QPair <double, Rect2d> > v;
    for( qint32 n = 0; n < numDetections; ++n )
    {
        v.append(qMakePair(detections[n].Confidence, Rect2d(detections[n].Left, detections[n].Top, detections[n].Width(), detections[n].Height())));
    }


    if (mode == Players)
    {
        for (auto& i : v)
        {
            if ((i.second.y + i.second.height / 2) < heightThreshold)
            {
                continue;
            }
            bool playerFound = false;
            if (!nonStructuredPlayers.empty())
            {
                QVector < QPair <double, PPlayer> > ious;
                for (auto& j : nonStructuredPlayers)
                {
                    ious.append(qMakePair(calculateIOU(i.second, j->rDet).iou, j));
                }
                sort(ious.begin(), ious.end(), [](auto& a, auto& b){return a.first > b.first;});
                auto max = ious.begin();
                // вернуть 0.25, добавить проверку площадей
                if (max->first > minIntercept)
                {
                    auto secondMax = max;
                    ++secondMax;
                    playerFound = true;
                    if (secondMax->first > minIntercept ||  max->second->interceptState > ProbablyIntercept)
                    {
                        if ((max->second->interceptState == DontMoveClose
                             || max->second->interceptState == MoveClose)
                                && max->second->lastTrackConfidence > trackerHighConfidence)
                        {
                            max->second->updated = true;
                            ++max->second->confirmed;
                            max->second->lastConfidence = i.first;
                        }
                        qDebug() << "TWO PEOPLE";
                    }
                    else
                    {
                        if (max->second->interceptState == Lost || max->second->interceptState == OutOfFrameLost)
                        {
                            max->second->speedUpdateCounter = 0;
                            max->second->interceptState = NoIntercept;
                        }
                        if (!max->second->speedUpdateCounter)
                        {
                            max->second->prevR = max->second->rUse;
                        }
                        else if (max->second->speedUpdateCounter == speedUpdateMax)
                        {
                            max->second->speed = Point2f(i.second.x / 2, i.second.y / 2)  - Point2f(max->second->prevR.x / 2, max->second->prevR.y / 2);
                            max->second->prevR = max->second->rUse;
                            max->second->speedUpdateCounter = 0;
                        }
                        ++max->second->speedUpdateCounter;

                        max->second->updated = true;
                        max->second->lastConfidence = i.first;
                        max->second->rDet = max->second->rUse = i.second;
                        ++max->second->confirmed;
                    }
                }

            }

            if (!playerFound)
            {
                bool findIntercepted = false;
                for (auto& interIt : interceptPlayers)
                {
                    Point2f foundPlayerCenter = Point2f (i.second.x + i.second.width / 2,
                                                         i.second.y + i.second.height / 2);
                    if (interIt.first.contains(i.second.tl())
                            || interIt.first.contains(i.second.br())
                            || interIt.first.contains(foundPlayerCenter))
                    {
                        if (interIt.second.size() == 2)
                        {
                            findIntercepted = true;

                            PPlayer first = interIt.second.first();
                            PPlayer second = interIt.second.last();
                            PPlayer tracked, nonTracked;
                            if (first->interceptState == MoveClose || first->interceptState == DontMoveClose)
                            {
                                tracked = first;
                                nonTracked = second;
                            }
                            else
                            {
                                tracked = second;
                                nonTracked = first;
                            }
                            resolveConflict(tracked, nonTracked, i.second);

                            Point2f trackedCenter = Point2f (tracked->rUse.x + tracked->rUse.width / 2,
                                                             tracked->rUse.y + tracked->rUse.height / 2);


                            Point2f trackedPredictedCenter = Point2f (tracked->predictedR.x + tracked->predictedR.width / 2,
                                                                      tracked->predictedR.y + tracked->predictedR.height / 2);

                            Point2f nonTrackedPredictedCenter = Point2f (nonTracked->predictedR.x + nonTracked->predictedR.width / 2,
                                                                         nonTracked->predictedR.y + nonTracked->predictedR.height / 2);

                            bool trackedCloserToTrackedPredicted = norm(trackedPredictedCenter - trackedCenter) < norm(nonTrackedPredictedCenter - trackedCenter) ? true : false;
                            bool foundCloserToNonTrackedPredicted = norm(nonTrackedPredictedCenter - foundPlayerCenter) < norm(trackedPredictedCenter - foundPlayerCenter) ? true : false;

                            auto hist = calculateBodyHueHist(inputImg, i.second);
                            imshow("window1", hist.first);
                            imshow("window3", tracked->origHist.first);
                            imshow("window4", nonTracked->origHist.first);
                            double trackedHistCorr = compareHist(tracked->origHist.second, hist.second, CV_COMP_CORREL);
                            double nonTrackedHistCorr = compareHist(nonTracked->origHist.second, hist.second, CV_COMP_CORREL);
                            if (nonTrackedHistCorr > trackedHistCorr)
                            {
                                qDebug() << nonTrackedHistCorr << trackedHistCorr << "NO SWAP";
                            }
                            else
                            {
                                qDebug() << nonTrackedHistCorr << trackedHistCorr << "SWAP";
                            }
                            if (!trackedCloserToTrackedPredicted && !foundCloserToNonTrackedPredicted)
                            {
                                auto tmpId = nonTracked->id;
                                nonTracked->id = tracked->id;
                                tracked->id = tmpId;
                            }

                            nonTracked->rUse = nonTracked->rDet = i.second;
                            nonTracked->updated = true;
                            ++nonTracked->confirmed;
                        }
                        for (auto interPlayer : interIt.second)
                        {
                            interPlayer->interceptState = NoIntercept;
                            if (!interPlayer->tracker.empty())
                            {
                                interPlayer->tracker->setDoLearning(true);
                            }
                            interPlayer->predictedR = Rect2d();
                        }
                        interceptPlayers.removeOne(interIt);
                        break;
                    }
                }
                if (!findIntercepted)
                {
                    nonStructuredPlayers.append(PPlayer::create());
                    nonStructuredPlayers.back()->lastConfidence = i.first;
                    nonStructuredPlayers.back()->rDet = nonStructuredPlayers.back()->rUse = i.second;
                    nonStructuredPlayers.back()->updated = true;
                    ++nonStructuredPlayers.back()->confirmed;
                }
            }


        }
    }

}


void PedestrianTracker::finishInitialization()
{
    if (mode == Players)
    {
        QLinkedList <PPlayer>::iterator i = nonStructuredPlayers.begin();
        while (i != nonStructuredPlayers.end())
        {
            if (i->data()->interceptState == Lost || i->data()->interceptState == OutOfFrameLost)
            {
                qDebug() << "handle LOST";
                ++i->data()->detNotFoundCount;
                if (i->data()->detNotFoundCount == 3)
                {
                    i = nonStructuredPlayers.erase(i);
                }
                else
                {
                    ++i;
                }
            }
            else
            {
                bool eraseBadTrack = qFuzzyCompare(i->data()->lastTrackConfidence, -1.);
                if (!i->data()->alreadyInited && !qFuzzyCompare(i->data()->lastTrackConfidence, -1.)
                        && i->data()->lastTrackConfidence < trackerHighConfidence
                        && i->data()->interceptState == NoIntercept)
                {
                    eraseBadTrack = true;
                }

                bool eraseBadDet = false;
                if (!i->data()->alreadyInited && i->data()->confirmed < 2
                        &&  i->data()->lastConfidence < confidenceThreshold)
                {
                    eraseBadDet = true;
                }

                if (eraseBadDet && eraseBadTrack)
                {
                    i = nonStructuredPlayers.erase(i);
                }
                else
                {
                    i->data()->confirmed = 0;
                    ++i;
                }
            }
        }
    }
    else
    {

    }
}


void PedestrianTracker::initTracker(Mat grayResized, PPlayer player)
{
    player->tracker = TrackerMOSSE::create();
    player->tracker->setLearningRate(0.25);
    player->tracker->setDoLearning(true);
    //player->tracker->setMaxDelta(Point2d(10, 10));
    //player->tracker->setCheckDelta(true);
    player->tracker->setThreshold(trackerThreshold);
    player->tracker->init(grayResized, player->rTrack);
}

void PedestrianTracker::trackPlayerInternal(Mat inputImg, PPlayer player, Mat grayResized, bool& erase)
{
    if (player->interceptState == Lost
            || player->interceptState == OutOfFrameLost)
    {
        return;
    }

    if (player->interceptState == MoveClose
            || player->interceptState == MoveFar)
    {
        player->predictedR.x += player->lastSpeed.x * 1.05;
        player->predictedR.y += player->lastSpeed.y * 1.05;
    }
    if (player->interceptState <= ProbablyIntercept
            || player->interceptState == DontMoveClose
            || player->interceptState == MoveClose)
    {

        if (!player->tracker->update(grayResized, player->rTrack))
        {
            qDebug() << "not found" << player->id << player->trackNotFoundCount
                     << player->tracker->getThreshold() << player->tracker->getConfidenceValue();
            if (!player->trackNotFoundCount)
            {
                initTracker(grayResized, player);
            }
            ++player->trackNotFoundCount;
            if (player->trackNotFoundCount > maxTrackNotFound)
            {
                player->interceptState = Lost;
            }
        }
        else
        {
            if (qFuzzyCompare(player->lastTrackConfidence, -1.))
            {
                player->lastTrackConfidence  = player->tracker->getConfidenceValue();
            }
            else
            {
                player->lastTrackConfidence  = trackConfidenceUpdateCoeff * player->lastTrackConfidence
                        + (1. - trackConfidenceUpdateCoeff) * player->tracker->getConfidenceValue();
            }

            player->rUse = player->rTrack;

            double expand = player->rTrack.width * expandBodyROICoeff;
            player->rUse.x += expand;
            player->rUse.width -= 2 * expand;
            player->rUse.x /= resampleCoeff;
            player->rUse.y /= resampleCoeff;
            player->rUse.width /= resampleCoeff;
            player->rUse.height /= resampleCoeff;
            if (player->rTrackCropped)
            {
                player->rUse.height /= bodyCropCoeff;
            }
            player->rDet = player->rUse;

            if (!player->speedUpdateCounter)
            {
                player->prevR = player->rUse;
            }
            else if (player->speedUpdateCounter == speedUpdateMax)
            {
                player->speed = Point2f(player->rUse.x, player->rUse.y)  - Point2f(player->prevR.x, player->prevR.y);
                player->prevR = player->rUse;
                player->speedUpdateCounter = 0;
            }
            ++player->speedUpdateCounter;

            player->trackNotFoundCount = 0;
            qDebug() << "tracked" << player->id <<  player->tracker->getConfidenceValue();
            qint32 horizontalCenter = player->rUse.x + player->rUse.width / 2;

            bool outOfFrameFlag = false;
            if (horizontalCenter < outOfFrameBorder)
            {
                outOfFrameFlag = true;
                emit outOfFrame(OutOfFrame::Left);
            }
            else if (horizontalCenter > inputImg.cols - outOfFrameBorder)
            {
                outOfFrameFlag = true;
                emit outOfFrame(OutOfFrame::Right);
            }
            if (outOfFrameFlag)
            {
                player->interceptState = OutOfFrameLost;
            }
        }

    }
}

QPair <Mat, Mat> PedestrianTracker::calculateBodyHueHist(Mat bodyImg, Rect2d r)
{
    Mat body;
    bodyImg(r).copyTo(body);
    Mat resizedBody;
    resize(body, resizedBody, Size(), resampleCoeff, resampleCoeff);
    Mat cropped;
    resizedBody(Rect(0, 0, resizedBody.cols, resizedBody.rows / 2)).copyTo(cropped);
    cvtColor(cropped, cropped, COLOR_BGR2HSV);
    std::vector <Mat> splitted;
    split(cropped, splitted);
    int histSize[] = {256};
    float hranges[] = { 0, 256 };
    const float* ranges[] = { hranges };
    Mat hist;
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0};

    calcHist( &splitted[0], 1, channels, Mat(), // do not use mask
            hist, 1, histSize, ranges,
            true, // the histogram is uniform
            false );

    normalize(hist, hist, 0, 1, NORM_MINMAX, -1, Mat() );

    //    int hist_w = 512; int hist_h = 400;
    //    int bin_w = cvRound( (double) hist_w/histSize[0] );

    //     Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
    //    normalize(hist, hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

    //    for( int i = 1; i < histSize[0]; i++ )
    //    {
    //        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist.at<float>(i-1)) ) ,
    //                         Point( bin_w*(i), hist_h - cvRound(hist.at<float>(i)) ),
    //                         Scalar( 255, 0, 0), 2, 8, 0  );
    //    }

    //    qDebug() << tt.nsecsElapsed() << "ELAPSED HSV";
    //    imshow("windows2", histImage);

    return qMakePair(splitted[0], hist);
}

void PedestrianTracker::resolveConflict(PedestrianTracker::PPlayer p1, PedestrianTracker::PPlayer p2, Rect2d foundPlayer)
{
    Ptr <TrackerCSRT> p1Tracker = TrackerCSRT::create();
    Ptr <TrackerCSRT> p2Tracker = TrackerCSRT::create();
    auto it = std::lower_bound(buffer->begin(), buffer->end(), p1->history.last().first, [](auto& a, auto& b) {return a.time < b;});
    p1Tracker->init(it->frame, p1->history.last().second);
    auto itStart = it;
    ++it;
    Rect2d p1PredictedRect;
    qint32 frameStep = 4;
    quint64 prevTime = it->time - 1;
    while (it->time > prevTime)
    {
        qDebug() << p1Tracker->update(it->frame, p1PredictedRect);
        rectangle (it->frame, p1PredictedRect, CV_RGB(255,0,0), 3);
        imshow("window2", it->frame);
        waitKey(0);
        it += frameStep;
    }
    qDebug() << calculateIOU(p1PredictedRect, foundPlayer).iou;
    //    p2Tracker->init(p2->history.last().first, p2->history.last().second);
    //    Rect2d p2PredictedRect;
    //    p2Tracker->update(inputImg, p2PredictedRect);
    //    qDebug() << calculateIOU(p1PredictedRect, foundPlayer).iou << calculateIOU(p2PredictedRect, foundPlayer).iou;
}

void PedestrianTracker::track(Mat _inputImg, quint64 time)
{    
    lastTime = time;
    inputImg = _inputImg;
    Mat gray;
    cvtColor(inputImg, gray, COLOR_BGR2GRAY);
    Mat grayResized;
    resize(gray, grayResized, Size(), resampleCoeff, resampleCoeff);
    switch (state)
    {

    case State::Detect:
    {
        readFrameInGPUMemory(inputImg);
        detectNet::Detection* detections = nullptr;
        const qint32 numDetections = net->Detect(imgCUDA, inputImg.cols, inputImg.rows, &detections, detectNet::OverlayFlags::OVERLAY_NONE);
        assignROIs(numDetections, detections);

        QLinkedList<PPlayer>::iterator i = nonStructuredPlayers.begin();
        while (i != nonStructuredPlayers.end())
        {
            bool erase = false;
            if (!i->data()->alreadyInited && i->data()->updated
                    && ((i->data()->confirmed > 2 && i->data()->lastConfidence > confidenceThreshold)
                        ||  i->data()->lastConfidence > 2 *confidenceThreshold))
            {
                initTrackerFirstly(grayResized, (*i));
                i->data()->alreadyInited = true;
            }
            if (i->data()->lastTrackConfidence > trackerHighConfidence)
            {
                trackPlayerInternal(inputImg, *i, grayResized, erase);
            }
            if (erase)
            {
                i = nonStructuredPlayers.erase(i);
            }
            else
            {
                ++i;
            }
        }

        ++detectCounter;

        if (detectCounter == detectDuration)
        {
            finishInitialization();
            QElapsedTimer t;
            t.start();
            Mat frameHistory;
            inputImg.copyTo(frameHistory);
            for (auto& player : nonStructuredPlayers)
            {
                savePlayerHistory(player);
                initTrackerFirstly(grayResized, player);
            }
            qDebug() << t.elapsed() << "ELAPSED INIT";
            state = Track;
            detectCounter = 0;
        }
        break;
    }
    case State::Track:
    {
        if (trackCounter < trackDuration)
        {
            if (mode == Players)
            {

                QElapsedTimer t;
                t.start();
                QLinkedList<PPlayer>::iterator i = nonStructuredPlayers.begin();
                while (i != nonStructuredPlayers.end())
                {
                    if (!trackCounter)
                    {
                        i->data()->rDet = Rect2d();
                        i->data()->alreadyInited = false;
                    }
                    bool erase = false;
                    trackPlayerInternal(inputImg, *i, grayResized, erase);
                    if (erase)
                    {
                        i = nonStructuredPlayers.erase(i);
                    }
                    else
                    {
                        ++i;
                    }
                }
                qDebug() << "ELAPSED" << t.elapsed();
                //                for (auto& i : nonStructuredPlayers)
                //                {
                //                    Rect r = i->rUse;
                //                    r.x /= 2;
                //                    r.y /= 2;
                //                    r.width /= 2;
                //                    r.height /= 2;
                //                    rectangle(grayResized, r, CV_RGB(255,0, 0), 3);
                //                }
                //                imshow("window5", grayResized);
            }
            ++trackCounter;
        }
        if (trackCounter == trackDuration)
        {
            trackCounter = 0;
            state = Detect;
        }
        break;
    }
    }
    interceptPlayers.append(checkPlayersIntercept());
}

void PedestrianTracker::initTrackerFirstly(Mat grayResized, PPlayer player)
{
    if (player->updated)
    {
        player->rTrackCropped = (player->rUse.height / player->rUse.width) > 1.5 ? true : false;
        player->rTrack.x = player->rUse.x * resampleCoeff;
        player->rTrack.y = player->rUse.y * resampleCoeff;
        player->rTrack.width = player->rUse.width * resampleCoeff;
        player->rTrack.height = player->rUse.height * resampleCoeff;
        double expand = player->rTrack.width * expandBodyROICoeff;
        player->rTrack.x -= expand;
        player->rTrack.width += 2 * expand;

        if (player->rTrackCropped)
        {
            player->rTrack.height *= bodyCropCoeff;
        }
        initTracker(grayResized, player);
    }
}

void PedestrianTracker::savePlayerHistory(PedestrianTracker::PPlayer p)
{
    if (p->interceptState <= ProbablyIntercept)
    {
        p->history.append(qMakePair(lastTime, p->rUse));
        if (p->history.size() > 3)
        {
            p->history.removeFirst();
        }
    }
}

void PedestrianTracker::drawROIs(Mat drawImg)
{
    if (mode == Players)
    {
        for (auto& i : nonStructuredPlayers)
        {
            auto p = i->rUse.tl();
            p.y -= 10;
            if (i->interceptState)
            {
                if (i->interceptState == DontMoveClose || i->interceptState == MoveClose)
                {
                    rectangle(drawImg, i->rUse, CV_RGB(0, 255, 0), 3);
                }
                else if (i->interceptState == Lost || i->interceptState == OutOfFrameLost)
                {
                    rectangle(drawImg, i->rUse, CV_RGB(0, 0, 255), 3);
                }
                else
                {
                    rectangle(drawImg, i->rUse, CV_RGB(255, 0, 0), 3);
                }
                if (!i->predictedR.empty())
                {
                    rectangle(drawImg, i->predictedR, CV_RGB(255, 255, 255), 3);
                }

                putText(drawImg, i->id.toString().left(5).toStdString(), i->rUse.tl(), FONT_HERSHEY_PLAIN, 2, CV_RGB(255, 0, 255));
            }
            else
            {
                rectangle(drawImg, i->rUse, CV_RGB(255, 255, 0), 3);
                putText(drawImg, i->id.toString().left(5).toStdString(), i->rUse.tl(), FONT_HERSHEY_PLAIN, 2, CV_RGB(255, 255, 255));
            }
        }
        for (auto& i : interceptPlayers)
        {
            rectangle(drawImg, i.first, CV_RGB(0, 0, 0), 3);
        }
    }
}


PedestrianTracker::InterceptCheckResult PedestrianTracker::intersectsAnyPedestrian(Point2f position, bool nonAprior)
{
    QMutexLocker lock(&mutex);
    QVector <QPair <double, PPlayer>> distance;
    InterceptCheckResult result;
    if (mode == Teams)
    {
        QMapIterator<TeamRole, PPlayer> attackIt(attackInfo);
        while (attackIt.hasNext())
        {
            result = handleInterceptCheck(position, attackIt.value(), distance, nonAprior);
            if (result.intercept)
            {
                result.role = attackIt.key();
                return result;
            }
        }

        QMapIterator<TeamRole, PPlayer> defenceIt(defenceInfo);
        while (defenceIt.hasNext())
        {
            result = handleInterceptCheck(position, defenceIt.value(), distance, nonAprior);
            if (result.intercept)
            {
                result.role = defenceIt.key();
                return result;
            }
        }
        for (auto& i : unconfirmedPlayers)
        {
            for (auto& j : i)
            {
                result = handleInterceptCheck(position, j, distance, nonAprior);
                if (result.intercept)
                {
                    return result;
                }
            }

        }
    }
    else
    {
        for (auto& i : nonStructuredPlayers)
        {
            result = handleInterceptCheck(position, i, distance, nonAprior);
            if (result.intercept)
            {
                return result;
            }
        }
    }

    auto min = std::min_element(distance.begin(), distance.end(), [](auto& a, auto& b) {return a.first < b.first;});
    result.p = min->second;
    result.distance = min->first;
    return result;

}

PedestrianTracker::PPlayer PedestrianTracker::findPlayerById(QUuid id, bool& ok)
{
    auto it = std::find_if(nonStructuredPlayers.begin(), nonStructuredPlayers.end(), [id](auto& a){return a->id == id;});
    if (it != nonStructuredPlayers.end())
    {
        ok = true;
        return *it;
    }
    else
    {
        return PPlayer();
    }




}

void PedestrianTracker::clear()
{
    if (mode == Players)
    {
        nonStructuredPlayers.clear();
        detectCounter = 0;
        trackCounter = 0;
        state = Detect;
    }
}

QVector<PedestrianTracker::LostPlayersArea> PedestrianTracker::checkPlayersIntercept()
{
    QVector <LostPlayersArea> result;
    QLinkedList <PPlayer> nonStructuredPlayersTmp = nonStructuredPlayers;
    QLinkedList <PPlayer>::iterator i = nonStructuredPlayersTmp.begin();

    bool pairFound = false;
    while (i != nonStructuredPlayersTmp.end() - 1)
    {
        pairFound = false;
        QLinkedList <PPlayer>::iterator j = (i + 1);
        while (j != nonStructuredPlayersTmp.end())
        {
            auto interResult = calculateIOU(i->data()->rUse, j->data()->rUse);


            if ((interResult.iou > minIOUIntercept / 3 || interResult.partR1 > minIntercept / 3 || interResult.partR2 > minIntercept / 3)
                    && i->data()->interceptState == NoIntercept
                    && j->data()->interceptState == NoIntercept)
            {
                if ((i->data()->confirmed >= 2 || !qFuzzyCompare(i->data()->lastTrackConfidence, -1.))
                        && (j->data()->confirmed >= 2 ||  !qFuzzyCompare(j->data()->lastTrackConfidence, -1.)))
                {
                    i->data()->origHist = calculateBodyHueHist(inputImg, i->data()->rUse);
                    j->data()->origHist = calculateBodyHueHist(inputImg, j->data()->rUse);
                    i->data()->interceptState = ProbablyIntercept;
                    j->data()->interceptState = ProbablyIntercept;
                }
            }
            if ((interResult.iou > minIOUIntercept || interResult.partR1 > minIntercept || interResult.partR2 > minIntercept)
                    && i->data()->interceptState <= ProbablyIntercept
                    && j->data()->interceptState <= ProbablyIntercept)
            {
                if ((i->data()->confirmed >= 3 || !qFuzzyCompare(i->data()->lastTrackConfidence, -1.))
                        && (j->data()->confirmed >= 3 ||  !qFuzzyCompare(j->data()->lastTrackConfidence, -1.)))
                {

                    Mat frameHistory;
                    inputImg.copyTo(frameHistory);
                    savePlayerHistory((*i));
                    savePlayerHistory((*j));

                    Point2f tl = i->data()->rUse.tl().x < j->data()->rUse.tl().x ? i->data()->rUse.tl() : j->data()->rUse.tl();
                    Point2f br = i->data()->rUse.br().x > j->data()->rUse.br().x ? i->data()->rUse.br() : j->data()->rUse.br();
                    tl.x -= 3 * i->data()->rUse.width;
                    br.x += 3 * i->data()->rUse.width;
                    Rect2d r (tl, br);

                    result.append(qMakePair(r, QVector <PPlayer> {*i, *j}));
                    double normSpeedFirst = norm(i->data()->speed);
                    bool firstBeforeSecond = i->data()->rUse.br().y > j->data()->rUse.br().y ? true : false;
                    if (normSpeedFirst > 10)
                    {
                        if (firstBeforeSecond)
                        {
                            i->data()->interceptState = MoveClose;
                        }
                        else
                        {
                            i->data()->interceptState = MoveFar;

                        }
                        i->data()->lastSpeed = i->data()->speed / speedUpdateMax;
                        i->data()->predictedR = i->data()->rUse;
                    }
                    else
                    {
                        if (firstBeforeSecond)
                        {
                            i->data()->interceptState = DontMoveClose;
                        }
                        else
                        {
                            i->data()->interceptState = DontMoveFar;
                        }
                        i->data()->predictedR = i->data()->rUse;
                    }
                    double normSpeedSecond = norm(j->data()->speed);
                    if (normSpeedSecond > 10)
                    {
                        if (!firstBeforeSecond)
                        {
                            j->data()->interceptState = MoveClose;
                        }
                        else
                        {
                            j->data()->interceptState = MoveFar;
                        }

                        j->data()->lastSpeed = j->data()->speed / speedUpdateMax;
                        j->data()->predictedR = j->data()->rUse;
                    }
                    else
                    {
                        if (!firstBeforeSecond)
                        {
                            j->data()->interceptState = DontMoveClose;
                        }
                        else
                        {
                            j->data()->interceptState = DontMoveFar;
                        }
                        j->data()->predictedR = j->data()->rUse;
                    }
                    if (!i->data()->tracker.empty())
                    {
                        i->data()->tracker->setDoLearning(false);
                    }
                    if (!j->data()->tracker.empty())
                    {
                        j->data()->tracker->setDoLearning(false);
                    }

                    pairFound = true;
                }

            }
            if (!pairFound)
            {
                ++j;
            }
            else
            {
                break;
            }
        }
        if (!pairFound)
        {
            ++i;
        }
        pairFound = false;
    }
    return result;
}



Point3f PedestrianTracker::evaluate3dPosition(Rect r, bool& ok)
{
    Point3f p;
    auto helper = CalibrationHelper::instance();
    Calibration::Position2D xy;
    xy.X = r.x + r.width / 2;
    xy.Y = r.y + r.height / 2;
    Calibration::RayAndPoint rp;
    if (helper.calculatePointOnGround(xy, rp))
    {
        ok = true;
        p.x = rp.Pos.X;
        p.y = rp.Pos.Y;
        p.z = rp.Pos.Z;
    }
    else
    {
        ok = false;
    }
    return p;
}
PedestrianTracker::IOUResult PedestrianTracker::calculateIOU(Rect2d r1, Rect2d r2)
{
    double xA = max(r1.x, r2.x);
    double yA = max(r1.y, r2.y);
    double xB = min(r1.x + r1.width, r2.x + r2.width);
    double yB = min(r1.y + r1.height, r2.y + r2.height);

    double interArea = max(0., xB - xA + 1) * max(0., yB - yA + 1);

    IOUResult res;
    res.iou = interArea / float(r1.area() + r2.area() - interArea);
    res.partR1 = interArea / r1.area();
    res.partR2 = interArea / r2.area();
    return res;
}

QPair <Point2f, bool> PedestrianTracker::checkIntersect(Player& p, Point2f& position, bool nonAprior)
{
    Point2f roiCenter = Point2f(p.rUse.x + p.rUse.width / 2, p.rUse.y + p.rUse.height / 2);
    Rect r = p.rUse;
    if (nonAprior)
    {
        r.x -= r.width / 2;
        r.width += r.width;
    }
    return qMakePair(norm(roiCenter - position), r.contains(position));
}

PedestrianTracker::InterceptCheckResult PedestrianTracker::handleInterceptCheck(Point2f& position, PPlayer player,
                                                                                QVector <QPair <double, PPlayer>>& distance, bool nonAprior)
{
    InterceptCheckResult res;
    auto result = checkIntersect(*player, position, nonAprior);
    double dist = sqrt(pow(result.first.x, 2) + pow(result.first.y, 2));
    if (result.second)
    {
        res.distance = dist;
        res.intercept = true;
        res.p = player;
    }
    else
    {
        distance.append(qMakePair(dist, player));
    }
    return res;
}
