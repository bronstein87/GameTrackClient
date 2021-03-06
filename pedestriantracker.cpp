#include "pedestriantracker.h"
#include <QDebug>
#include <QElapsedTimer>
#include <opencvhelpfunction.h>

PedestrianTracker::PedestrianTracker(qint32 width, qint32 height, QObject *parent) : DetectNetBase(width, height, parent)
{
    //    attackInfo.insert(TeamRole::Batter, PPlayer::create());
    //    attackInfo.insert(TeamRole::FirstBase, PPlayer::create());
    //    attackInfo.insert(TeamRole::SecondBase, PPlayer::create());
    //    attackInfo.insert(TeamRole::ThirdBase, PPlayer::create());

    //    defenceInfo.insert(TeamRole::Pitcher, PPlayer::create());
    //    defenceInfo.insert(TeamRole::Catcher, PPlayer::create());
    //    defenceInfo.insert(TeamRole::FirstDefence, PPlayer::create());
    //    defenceInfo.insert(TeamRole::SecondDefence, PPlayer::create());
    //    defenceInfo.insert(TeamRole::ThirdDefence, PPlayer::create());
    //    defenceInfo.insert(TeamRole::ShortStop, PPlayer::create());
    //    defenceInfo.insert(TeamRole::LeftFielder, PPlayer::create());
    //    defenceInfo.insert(TeamRole::CenterFielder, PPlayer::create());
    //    defenceInfo.insert(TeamRole::RightFielder, PPlayer::create());

    positionCoordinates.insert(GamePositions::Invalid, qMakePair(QString("Invalid"),
                                                                 Point3f(std::numeric_limits <float>::max(),
                                                                         std::numeric_limits <float>::max(),
                                                                         std::numeric_limits <float>::max())));
    positionCoordinates.insert(GamePositions::Home, qMakePair(QString("Home"), Point3f(0, 0, 0)));
    positionCoordinates.insert(GamePositions::Umpire, qMakePair(QString("Umpire"), Point3f(-1.72, -1.72, 0)));
    positionCoordinates.insert(GamePositions::LeftSquare, qMakePair(QString("LeftSquare"), Point3f(-1, 1, 0)));
    positionCoordinates.insert(GamePositions::RightSquare, qMakePair(QString("RightSquare"), Point3f(1, -1, 0)));
    positionCoordinates.insert(GamePositions::FirstBase, qMakePair(QString("FirstBase"), Point3f(27.04, 0.04, 0)));
    positionCoordinates.insert(GamePositions::SecondBase, qMakePair(QString("SecondBase"), Point3f(27.23, 27.27, 0)));
    positionCoordinates.insert(GamePositions::ShortStop, qMakePair(QString("ShortStop"), Point3f(14.5, 27.4, 0)));
    positionCoordinates.insert(GamePositions::ThirdBase, qMakePair(QString("ThirdBase"), Point3f(0.005, 27.07, 0)));
    positionCoordinates.insert(GamePositions::LeftField, qMakePair(QString("LeftField"), Point3f(16, 62, 0)));
    positionCoordinates.insert(GamePositions::CenterField, qMakePair(QString("CenterField"), Point3f(50, 50, 0)));
    positionCoordinates.insert(GamePositions::RightField, qMakePair(QString("RightField"), Point3f(62, 16, 0)));
    positionCoordinates.insert(GamePositions::Pitch, qMakePair(QString("Pitch"), Point3f(12.9145, 13.4031, 0.2717)));
    positionCoordinates.insert(GamePositions::UmpireFirstBase, qMakePair(QString("UmpireFirstBase"), Point3f(38.4839,	-0.8891, 0)));
    positionCoordinates.insert(GamePositions::UmpireSecondBase, qMakePair(QString("UmpireSecondBase"), Point3f(-0.9813, 38.5129, -0.0063)));
    positionCoordinates.insert(GamePositions::UmpirePitchFirst, qMakePair(QString("UmpirePitchFirst"), Point3f(18.2, 21, 0)));
    positionCoordinates.insert(GamePositions::UmpirePitchSecond, qMakePair(QString("UmpirePitchSecond"), Point3f(21, 18.2, 0)));
    positionCoordinates.insert(GamePositions::CouchFirst, qMakePair(QString("CouchFirst"), Point3f(24.2, -4.5, 0)));
    positionCoordinates.insert(GamePositions::CouchSecond, qMakePair(QString("CouchSecond"), Point3f(-4.5, 24.2, 0)));


}


void PedestrianTracker::assignROIs(const qint32 numDetections, detectNet::Detection* detections)
{
    //QMutexLocker lock(&mutex);
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
            checkRectangleSize(i.second, inputImg.cols, inputImg.rows);
            if ((i.second.y + i.second.height / 2) < heightThreshold)
            {
                continue;
            }
            bool playerFound = false;
            if (!nonStructuredPlayers.empty())
            {
                QVector < QPair <IOUResult, PPlayer> > ious;
                for (auto& j : nonStructuredPlayers)
                {
                    ious.append(qMakePair(calculateIOU(i.second, j->rDet), j));
                }
                sort(ious.begin(), ious.end(), [](auto& a, auto& b){return a.first.iou > b.first.iou;});
                auto max = ious.begin();
                // вернуть 0.25, добавить проверку площадей
                qDebug() << "IOU"  << positionCoordinates[max->second->position].first <<
                            max->first.iou <<  max->first.partR1 <<  max->first.partR2;
                if (max->first.iou > minIOUIntersect
                        || (max->first.partR1 > minIOUIntersect
                        && max->first.partR2 > minIOUIntersect))
                {
                    auto secondMax = max;
                    ++secondMax;

                    if (secondMax->first.iou > minIntersect)
                    {
                        if ((max->second->intersectState == DontMoveClose
                             || max->second->intersectState == MoveClose)
                                && max->second->lastTrackConfidence > trackerHighConfidence)
                        {
                            max->second->updated = true;
                            ++max->second->confirmed;
                            max->second->lastConfidence = i.first;
                            playerFound = true;
                        }
                        if (max->first.iou > 0.9 && abs(secondMax->first.partR1 - secondMax->first.partR2) > 0.5)
                        {
                            max->second->updated = true;
                            ++max->second->confirmed;
                            max->second->lastConfidence = i.first;
                            playerFound = true;
                        }
                    }
                    else
                    {
                        playerFound = true;
                        if (max->second->intersectState == Lost || max->second->intersectState == OutOfFrameLost)
                        {
                            max->second->speedUpdateCounter = 0;
                            max->second->intersectState = NoIntersect;
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
                        qDebug() << "found player" << positionCoordinates[max->second->position].first
                                                             << max->second->rUse.x
                                                             << max->second->rUse.y
                                                             << max->second->rUse.width
                                                             << max->second->rUse.height;
                    }
                }
                else
                {
                    if (max->first.iou > 0.1 && max->second->position == GamePositions::Pitch)
                    {
                        max->second->updated = true;
                        ++max->second->confirmed;
                        max->second->lastConfidence = i.first;
                        playerFound = true;
                    }
                    qDebug() << "low intercept" << max->first.iou << max->first.partR1 << max->first.partR2 << positionCoordinates[max->second->position].first;
                }

            }

            if (!playerFound)
            {
                bool findIntersected = false;
                for (auto& interIt : intersectPlayers)
                {
                    Point2f foundPlayerCenter = Point2f (i.second.x + i.second.width / 2,
                                                         i.second.y + i.second.height / 2);
                    if (interIt.first.contains(i.second.tl())
                            || interIt.first.contains(i.second.br())
                            || interIt.first.contains(foundPlayerCenter))
                    {
                        if (interIt.second.size() == 2)
                        {
                            findIntersected = true;

                            PPlayer first = interIt.second.first();
                            PPlayer second = interIt.second.last();
                            PPlayer tracked, nonTracked;
                            if (first->intersectState == MoveClose || first->intersectState == DontMoveClose)
                            {
                                tracked = first;
                                nonTracked = second;
                            }
                            else
                            {
                                tracked = second;
                                nonTracked = first;
                            }
                            //resolveConflict(tracked, nonTracked, i.second);

                            Point2f trackedCenter = Point2f (tracked->rUse.x + tracked->rUse.width / 2,
                                                             tracked->rUse.y + tracked->rUse.height / 2);


                            Point2f trackedPredictedCenter = Point2f (tracked->predictedR.x + tracked->predictedR.width / 2,
                                                                      tracked->predictedR.y + tracked->predictedR.height / 2);

                            Point2f nonTrackedPredictedCenter = Point2f (nonTracked->predictedR.x + nonTracked->predictedR.width / 2,
                                                                         nonTracked->predictedR.y + nonTracked->predictedR.height / 2);

                            bool trackedCloserToTrackedPredicted = norm(trackedPredictedCenter - trackedCenter) < norm(nonTrackedPredictedCenter - trackedCenter) ? true : false;
                            bool foundCloserToNonTrackedPredicted = norm(nonTrackedPredictedCenter - foundPlayerCenter) < norm(trackedPredictedCenter - foundPlayerCenter) ? true : false;

                            auto hist = calculateBodyHueHist(inputImg, i.second);
                            //                            imshow("window1", hist.first);
                            //                            imshow("window3", tracked->origHist.first);
                            //                            imshow("window4", nonTracked->origHist.first);
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
                                auto tmpMoves = nonTracked->moves;
                                auto tmpPosition = nonTracked->position;
                                auto tmpColor = nonTracked->color;
                                nonTracked->id = tracked->id;
                                nonTracked->moves = tracked->moves;
                                nonTracked->position = tracked->position;
                                nonTracked->color = tracked->color;
                                tracked->id = tmpId;
                                tracked->moves = tmpMoves;
                                tracked->position = tmpPosition;
                                tracked->color = tmpColor;
                            }

                            nonTracked->rUse = nonTracked->rDet = i.second;
                            nonTracked->updated = true;
                            ++nonTracked->confirmed;
                        }
                        for (auto interPlayer : interIt.second)
                        {
                            interPlayer->intersectState = NoIntersect;
                            if (!interPlayer->tracker.empty())
                            {
                                interPlayer->tracker->setDoLearning(true);
                            }
                            interPlayer->predictedR = Rect2d();
                        }
                        intersectPlayers.removeOne(interIt);
                        break;
                    }
                }
                if (!findIntersected)
                {
                    nonStructuredPlayers.append(PPlayer::create());
                    nonStructuredPlayers.back()->lastConfidence = i.first;
                    nonStructuredPlayers.back()->rDet = nonStructuredPlayers.back()->rUse = i.second;
                    nonStructuredPlayers.back()->updated = true;
                    RNG rng(nonStructuredPlayers.size());
                    nonStructuredPlayers.back()->color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
                    ++nonStructuredPlayers.back()->confirmed;
                    qDebug() << "NEW PLAYER" << nonStructuredPlayers.back()->id
                             << nonStructuredPlayers.back()->rUse.x
                             << nonStructuredPlayers.back()->rUse.y
                             << nonStructuredPlayers.back()->rUse.width
                             << nonStructuredPlayers.back()->rUse.height;
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
            if (i->data()->intersectState == Lost || i->data()->intersectState == OutOfFrameLost)
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
                        && i->data()->intersectState == NoIntersect)
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


bool PedestrianTracker::initTracker(Mat grayResized, PPlayer player)
{
    player->tracker = TrackerMOSSE::create();
    player->tracker->setLearningRate(0.25);
    player->tracker->setDoLearning(true);
    //player->tracker->setMaxDelta(Point2d(10, 10));
    //player->tracker->setCheckDelta(true);
    player->tracker->setThreshold(trackerThreshold);
    bool inited = player->tracker->init(grayResized, player->rTrack);
    inited = inited && (player->rTrack.width != 0 && player->rTrack.height != 0);
    return inited;
}

void PedestrianTracker::trackPlayerInternal(Mat inputImg, PPlayer player, Mat grayResized, bool& erase)
{
    if (player->intersectState == Lost
            || player->intersectState == OutOfFrameLost)
    {
        return;
    }

    if (player->intersectState == MoveClose
            || player->intersectState == MoveFar)
    {
        player->predictedR.x += player->lastSpeed.x * 1.05;
        player->predictedR.y += player->lastSpeed.y * 1.05;
    }
    if (player->intersectState <= ProbablyIntersect
            || player->intersectState == DontMoveClose
            || player->intersectState == MoveClose)
    {
        if (!player->tracker->update(grayResized, player->rTrack))
        {
            qDebug() << "not found" << player->id << player->trackNotFoundCount
                     << player->tracker->getThreshold()
                     << player->tracker->getConfidenceValue()
                     << positionCoordinates[player->position].first;
            if (player->trackNotFoundCount < tryToTrack)
            {
                initTracker(grayResized, player);
            }
            ++player->trackNotFoundCount;
            if (player->trackNotFoundCount > maxTrackNotFound)
            {
                player->intersectState = Lost;
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
            player->rUse.width = player->rUse.width / (1 + 2 * expandBodyROICoeff);
            player->rUse.x = player->rUse.x + (player->rUse.width * expandBodyROICoeff);
            player->rUse.x /= resampleCoeff;
            player->rUse.y /= resampleCoeff;
            player->rUse.width /= resampleCoeff;
            player->rUse.height /= resampleCoeff;
            if (player->rTrackCropped)
            {
                player->rUse.height /= bodyCropCoeff;
            }
            checkRectangleSize(player->rUse, inputImg.cols, inputImg.rows);
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
            qDebug() << "tracked" << player->id <<  player->tracker->getConfidenceValue() <<  positionCoordinates[player->position].first;
            qint32 horizontalCenter = player->rUse.x + player->rUse.width / 2;

            bool outOfFrameFlag = false;
            if (horizontalCenter < outOfFrameBorder)
            {
                outOfFrameFlag = true;
                erase = true;
                emit outOfFrame(msg::OutOfFrame::LeftOut);
            }
            else if (horizontalCenter > inputImg.cols - outOfFrameBorder)
            {
                outOfFrameFlag = true;
                erase = true;
                emit outOfFrame(msg::OutOfFrame::RightOut);
            }
            if (outOfFrameFlag)
            {
                player->intersectState = OutOfFrameLost; //  if erase, we don't need this status anymore
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
    resizedBody(cv::Rect(0, 0, resizedBody.cols, resizedBody.rows / 2)).copyTo(cropped);
    cvtColor(cropped, cropped, COLOR_BGR2HSV);
    std::vector <Mat> splitted;
    split(cropped, splitted);
    int histSize[] = {128};
    float hranges[] = { 0, 128 };
    const float* ranges[] = { hranges };
    Mat hist;
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0};

    calcHist( &splitted[0], 1, channels, Mat(), // do not use mask
            hist, 1, histSize, ranges,
            true, // the histogram is uniform
            false );

    normalize(hist, hist, 0, 1, NORM_MINMAX, -1, Mat() );

    return qMakePair(splitted[0], hist);
}

void PedestrianTracker::resolveConflict(PedestrianTracker::PPlayer p1, PedestrianTracker::PPlayer p2, Rect2d foundPlayer)
{
    Q_UNUSED(p2);
    Ptr <TrackerCSRT> p1Tracker = TrackerCSRT::create();
    //Ptr <TrackerCSRT> p2Tracker = TrackerCSRT::create();
    auto it = std::lower_bound(buffer->begin(), buffer->end(), p1->history.first, [](auto& a, auto& b) {return a.time < b;});
    p1Tracker->init(it->frame, p1->history.second);
    ++it;
    Rect2d p1PredictedRect;
    qint32 frameStep = 6;
    while (it->time != lastTime)
    {
        qDebug() << p1Tracker->update(it->frame, p1PredictedRect) << lastTime - it->time;
        rectangle (it->frame, p1PredictedRect, CV_RGB(255, 0, 0), 3);
        //imshow("window2", it->frame);
        //waitKey(0);
        qint32 i = 0;
        while (it != buffer->end() - 1 && i < frameStep)
        {
            ++it;
            ++i;
        }
    }
    qDebug() << calculateIOU(p1PredictedRect, foundPlayer).iou;
}

bool PedestrianTracker::assignPosition(PPlayer player, bool& changed)
{
    bool ok;
    MovePosition mPos = evaluate3dPosition(player->rUse, ok);
    player->moves.append(qMakePair(lastTime, mPos));
    Q_ASSERT(ok);
    auto min = std::min_element(positionCoordinates.begin(), positionCoordinates.end(), [mPos](auto& p1, auto& p2)
    {
        return norm(p1.second - mPos.onSpace) < norm(p2.second - mPos.onSpace);
    });
    double distance = norm(mPos.onSpace - min.value().second);

    if (player->position == GamePositions::Invalid ||  distance < 1.5)
    {
        if (min.key() == GamePositions::Pitch && distance > maxPitchDistance)
        {
            QString name = min.value().first;
            min = std::min_element(positionCoordinates.begin(), positionCoordinates.end(), [mPos, name](auto& p1, auto& p2)
            {
                if (p1.first == name)
                {
                    return false;
                }
                else if (p2.first == name)
                {
                    return true;
                }
                else
                {
                    return norm(p1.second - mPos.onSpace) < norm(p2.second - mPos.onSpace);
                }

            });
        }

        if (player->position != min.key())
        {
            changed = true;
        }
        player->position = min.key();
    }
    return ok;

}

void PedestrianTracker::correctPlayerHeight(PPlayer player)
{
    CalibrationHelper& helper = CalibrationHelper::instance();
    Calibration::Position pos;
    Calibration::Position2D pos2d;
    pos.X = player->moves.back().second.onSpace.x;
    pos.Y = player->moves.back().second.onSpace.y;
    pos.Z = playerMediumHeight;
    helper.correctHumanHeight(pos, pos2d);
    player->rUse.height -= pos2d.Y - player->rUse.y ;
    player->rUse.y = pos2d.Y;
    player->moves.back().second.height = playerMediumHeight;
}

void PedestrianTracker::track(Mat _inputImg, quint64 time)
{    
    QElapsedTimer t, t1, t2, t3, t4, t5, t6;
    t.start();
    net->interceptCoeff = minIntersect; // correct intercept
    lastTime = time;
    inputImg = _inputImg;

    t2.start();
    Mat gray;
    cvtColor(inputImg, gray, COLOR_BGR2GRAY);
    Mat grayResized;
    resize(gray, grayResized, Size(), resampleCoeff, resampleCoeff);
    qDebug() << "PREPARE IMAGE" << t2.elapsed();

    qDebug() << "CURRENT STATE" << state;
    switch (state)
    {

    case State::Detect:
    {

        t5.start();
        cv::cuda::GpuMat input;
        cv::cuda::GpuMat input1;
        cv::cuda::GpuMat input2;
        input.upload(inputImg);
        cv::cuda::cvtColor(input, input1, COLOR_BGR2RGBA);
        input1.convertTo(input2, CV_32FC4);
        qDebug() << "TEST ELAPSED" << t5.elapsed() << input.cols << input.rows <<input.type() << input2.data;
        t1.start();
        detectNet::Detection* detections = nullptr;
        const qint32 numDetections = net->Detect((float*)input2.data , inputImg.cols, inputImg.rows, &detections, detectNet::OverlayFlags::OVERLAY_NONE);
        qDebug() << "DETECT ELAPSED" << t1.elapsed() << numDetections;
        t6.start();
        assignROIs(numDetections, detections);
        qDebug() << "assign elapsed" << t6.elapsed();
        QLinkedList<PPlayer>::iterator i = nonStructuredPlayers.begin();
        t3.start();
        while (i != nonStructuredPlayers.end())
        {
            bool erase = false;
            if (!i->data()->alreadyInited && i->data()->updated
                    && ((i->data()->confirmed > 2 && i->data()->lastConfidence > confidenceThreshold)
                        ||  i->data()->lastConfidence > 2 * confidenceThreshold))
            {
                if (initTrackerFirstly(grayResized, (*i)))
                {
                    i->data()->alreadyInited = true;
                }
                else
                {
                    erase = true;
                }
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
        qDebug() << "track elapsed" << t3.elapsed() ;
        t4.start();
        for (auto& i : nonStructuredPlayers)
        {
            if (i->updated)
            {
                bool changed = false;
                if (assignPosition(i, changed))
                {
                    if (i->moves.back().second.height > playerHeightThreshold)
                    {

                        correctPlayerHeight(i);
                    }
                }
            }
        }
        qDebug() << "position elapsed" << t4.elapsed();
        ++detectCounter;

        if (detectCounter == detectDuration)
        {

            finishInitialization();
            QElapsedTimer tt;
            tt.start();
            QLinkedList<PPlayer>::iterator i = nonStructuredPlayers.begin();
            while (i != nonStructuredPlayers.end())
            {
                if (!i->data()->updated && !i->data()->alreadyInited
                        && i->data()->lastTrackConfidence < trackerHighConfidence)
                {
                    i = nonStructuredPlayers.erase(i);
                }
                else
                {
                    if (initTrackerFirstly(grayResized, *i))
                    {
                        savePlayerHistory(*i);
                        ++i;
                    }
                    else
                    {
                        i = nonStructuredPlayers.erase(i);
                    }
                }


            }
            qDebug() << tt.elapsed() << "ELAPSED INIT";
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
                for (auto& i : nonStructuredPlayers)
                {
                    if (i->updated)
                    {
                        bool changed = false;
                        assignPosition(i, changed);
                    }
                }


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
    qDebug() << "ELAPSED" << t.elapsed();
    intersectPlayers.append(checkPlayersIntersect());
}


bool PedestrianTracker::initTrackerFirstly(Mat grayResized, PPlayer player)
{
    qDebug() << "firstly" << player->id << positionCoordinates[player->position].first;
    player->rTrackCropped = (player->rUse.height / player->rUse.width) > bodyWidthHeightProportion ? true : false;
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
    return initTracker(grayResized, player);
}

void PedestrianTracker::savePlayerHistory(PedestrianTracker::PPlayer p)
{
    if (p->intersectState <= ProbablyIntersect)
    {
        p->history = qMakePair(lastTime, p->rUse);
    }
}

void PedestrianTracker::drawROIs(Mat drawImg)
{
    if (mode == Players)
    {
        for (auto& i : nonStructuredPlayers)
        {
//            if (i->moves.size() > 2)
//            {
//                for (qint32 j = 1; j < i->moves.size(); ++j)
//                {
//                    line(drawImg, i->moves[j - 1].second.onCam, i->moves[j].second.onCam, i->color, 10);
//                }
//            }

            auto p = i->rUse.tl();
            p.y -= 10;
            if (i->intersectState)
            {

                if (i->intersectState == DontMoveClose || i->intersectState == MoveClose)
                {
                    rectangle(drawImg, i->rUse, CV_RGB(0, 255, 0), 3);
                }
                else if (i->intersectState == Lost || i->intersectState == OutOfFrameLost)
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

                putText(drawImg, positionCoordinates[i->position].first.toStdString(), Point2f(i->rUse.tl().x, i->rUse.tl().y - 20),
                        FONT_HERSHEY_PLAIN, 2, CV_RGB(255, 0, 255), 3);
            }
            else
            {
                rectangle(drawImg, i->rUse, CV_RGB(255, 255, 0), 3);
                putText(drawImg, positionCoordinates[i->position].first.toStdString(), Point2f(i->rUse.tl().x, i->rUse.tl().y - 20),
                        FONT_HERSHEY_PLAIN, 2, CV_RGB(255, 255, 255), 3);
            }
        }
        for (auto& i : intersectPlayers)
        {
            rectangle(drawImg, i.first, CV_RGB(0, 0, 0), 3);
        }
    }
}


PedestrianTracker::IntersectCheckResult PedestrianTracker::intersectsAnyPedestrian(Point2f position, bool nonAprior)
{
    QMutexLocker lock(&mutex);
    QVector <QPair <double, PPlayer>> distance;
    IntersectCheckResult result;
    if (mode == Teams)
    {
        //        QMapIterator<TeamRole, PPlayer> attackIt(attackInfo);
        //        while (attackIt.hasNext())
        //        {
        //            result = handleIntersectCheck(position, attackIt.value(), distance, nonAprior);
        //            if (result.intersect)
        //            {
        //                result.role = attackIt.key();
        //                return result;
        //            }
        //        }

        //        QMapIterator<TeamRole, PPlayer> defenceIt(defenceInfo);
        //        while (defenceIt.hasNext())
        //        {
        //            result = handleIntersectCheck(position, defenceIt.value(), distance, nonAprior);
        //            if (result.intersect)
        //            {
        //                result.role = defenceIt.key();
        //                return result;
        //            }
        //        }
        //        for (auto& i : unconfirmedPlayers)
        //        {
        //            for (auto& j : i)
        //            {
        //                result = handleIntersectCheck(position, j, distance, nonAprior);
        //                if (result.intersect)
        //                {
        //                    return result;
        //                }
        //            }

        //        }
    }
    else
    {
        for (auto& i : nonStructuredPlayers)
        {
            result = handleIntersectCheck(position, i, distance, nonAprior);
            if (result.intersect)
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
        //unconfirmedPlayers.clear();
        nonStructuredPlayers.clear();
        detectCounter = 0;
        trackCounter = 0;
        state = Detect;
    }
}

//PedestrianTracker::PedestrianTrackerState PedestrianTracker::saveState()
//{
//    PedestrianTrackerState state;
//    for (PPlayer& i : nonStructuredPlayers)
//    {
//        state.nonStructuredPlayers.append(PPlayer());
//        *state.nonStructuredPlayers.back() = *i;
//        *state.nonStructuredPlayers.back()->tracker = *i->tracker;
//        i->origHist.first.copyTo(state.nonStructuredPlayers.back()->origHist.first);
//        i->origHist.second.copyTo(state.nonStructuredPlayers.back()->origHist.second);
//    }

//    //    struct PedestrianTrackerState
//    //    {
//    //        QLinkedList <PPlayer> nonStructuredPlayers;
//    //        QVector <LostPlayersArea> intersectPlayers;
//    //        TrackMode mode = Players;
//    //        State state = Detect;
//    //        Mat inputImg;
//    //        quint64 lastTime;
//    //        qint32 detectCounter = 0;
//    //        qint32 trackCounter = 0;
//    //    };
//}

//void PedestrianTracker::resetState(PedestrianTracker::PedestrianTrackerState &state)
//{

//}

QVector<PedestrianTracker::LostPlayersArea> PedestrianTracker::checkPlayersIntersect()
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

            if ((interResult.iou > minIOUIntersect / 3 || interResult.partR1 > minIntersect / 3 || interResult.partR2 > minIntersect / 3)
                    && i->data()->intersectState == NoIntersect
                    && j->data()->intersectState == NoIntersect)
            {
                if ((i->data()->confirmed >= 2 || !qFuzzyCompare(i->data()->lastTrackConfidence, -1.))
                        && (j->data()->confirmed >= 2 ||  !qFuzzyCompare(j->data()->lastTrackConfidence, -1.)))
                {
                    qDebug() << "get hue"
                             << positionCoordinates[i->data()->position].first  << i->data()->id
                             << positionCoordinates[j->data()->position].first << j->data()->id;
                    i->data()->origHist = calculateBodyHueHist(inputImg, i->data()->rUse);
                    j->data()->origHist = calculateBodyHueHist(inputImg, j->data()->rUse);
                    i->data()->intersectState = ProbablyIntersect;
                    j->data()->intersectState = ProbablyIntersect;
                    qDebug() << "got hue";
                }
            }

            if ((interResult.iou > minIOUIntersect || interResult.partR1 > minIntersect || interResult.partR2 > minIntersect)
                    && i->data()->intersectState <= ProbablyIntersect
                    && j->data()->intersectState <= ProbablyIntersect)
            {
                if ((i->data()->confirmed >= 3 || !qFuzzyCompare(i->data()->lastTrackConfidence, -1.))
                        && (j->data()->confirmed >= 3 ||  !qFuzzyCompare(j->data()->lastTrackConfidence, -1.)))
                {

                    savePlayerHistory((*i));
                    savePlayerHistory((*j));

                    Point2f tl = i->data()->rUse.tl().x < j->data()->rUse.tl().x ? i->data()->rUse.tl() : j->data()->rUse.tl();
                    Point2f br = i->data()->rUse.br().x > j->data()->rUse.br().x ? i->data()->rUse.br() : j->data()->rUse.br();
                    tl.x -= 6 * i->data()->rUse.width;
                    br.x += 6 * i->data()->rUse.width;
                    Rect2d r (tl, br);

                    result.append(qMakePair(r, QVector <PPlayer> {*i, *j}));
                    double normSpeedFirst = norm(i->data()->speed);
                    bool firstBeforeSecond = i->data()->rUse.br().y > j->data()->rUse.br().y ? true : false;


                    if (normSpeedFirst > speedThreshold)
                    {
                        if (firstBeforeSecond)
                        {
                            i->data()->intersectState = MoveClose;
                        }
                        else
                        {
                            i->data()->intersectState = MoveFar;

                        }
                        i->data()->lastSpeed = i->data()->speed / speedUpdateMax;
                        i->data()->predictedR = i->data()->rUse;
                    }
                    else
                    {
                        if (firstBeforeSecond)
                        {
                            i->data()->intersectState = DontMoveClose;
                        }
                        else
                        {
                            i->data()->intersectState = DontMoveFar;
                        }
                        i->data()->predictedR = i->data()->rUse;
                    }
                    double normSpeedSecond = norm(j->data()->speed);

                    if (normSpeedSecond > speedThreshold)
                    {
                        if (!firstBeforeSecond)
                        {
                            j->data()->intersectState = MoveClose;
                        }
                        else
                        {
                            j->data()->intersectState = MoveFar;
                        }

                        j->data()->lastSpeed = j->data()->speed / speedUpdateMax;
                        j->data()->predictedR = j->data()->rUse;
                    }
                    else
                    {
                        if (!firstBeforeSecond)
                        {
                            j->data()->intersectState = DontMoveClose;
                        }
                        else
                        {
                            j->data()->intersectState = DontMoveFar;
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


PedestrianTracker::MovePosition PedestrianTracker::evaluate3dPosition(cv::Rect r, bool& ok)
{
    Point3f p;
    CalibrationHelper& helper = CalibrationHelper::instance();
    Calibration::Position2D xy;
    xy.X = r.x + r.width / 2;
    xy.Y = r.y + r.height;
    Calibration::RayAndPoint rp;
    Calibration::Position xyzHeight;
    if (helper.calculatePointOnGround(xy, rp))
    {
        ok = true;
        p.x = rp.Pos.X;
        p.y = rp.Pos.Y;
        p.z = rp.Pos.Z;
        Calibration::Position2D xyHead;
        xyHead.X = r.x + r.width / 2;
        xyHead.Y = r.y;

        if (helper.getZForHuman(xyHead, rp.Pos, xyzHeight))
        {
            qDebug() << "HEIGHT" << xyzHeight.Z;
        }
    }
    else
    {
        ok = false;
    }
    MovePosition mPos;
    mPos.onSpace = p;
    mPos.onCam = Point2f(xy.X, xy.Y);
    mPos.height = xyzHeight.Z;
    return mPos;
}


QPair <Point2f, bool> PedestrianTracker::checkIntersect(Player& p, Point2f& position, bool nonAprior)
{
    Point2f roiCenter = Point2f(p.rUse.x + p.rUse.width / 2, p.rUse.y + p.rUse.height / 2);
    cv::Rect r = p.rUse;
    if (nonAprior)
    {
        r.x -= r.width / 2;
        r.width += r.width;
    }
    return qMakePair(norm(roiCenter - position), r.contains(position));
}

PedestrianTracker::IntersectCheckResult PedestrianTracker::handleIntersectCheck(Point2f& position, PPlayer player,
                                                                                QVector <QPair <double, PPlayer>>& distance, bool nonAprior)
{
    IntersectCheckResult res;
    auto result = checkIntersect(*player, position, nonAprior);
    double dist = sqrt(pow(result.first.x, 2) + pow(result.first.y, 2));
    if (result.second)
    {
        res.distance = dist;
        res.intersect = true;
        res.p = player;
    }
    else
    {
        distance.append(qMakePair(dist, player));
    }
    return res;
}
