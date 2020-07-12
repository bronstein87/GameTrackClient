#include "ballrecognizerpp.h"

BallRecognizerPP::BallRecognizerPP(QObject *parent) : QObject(parent)
{
    cv::namedWindow("window",WINDOW_NORMAL);
    cv::namedWindow("window2",WINDOW_NORMAL);
}

BallRecognizerPP::State BallRecognizerPP::recognize(Mat frame, qint64 time)
{
    if (rois.mainSearchRect.empty() ||
            rois.trackFirstRect.empty() ||
            rois.trackSecondRect.empty())
    {
        emit errorOccured("Не заданы ROI.");
    }

#ifdef DEBUG_BALL_REC
    //        params.minArea = 10;/*15*//*30*/;
    //        params.maxArea = 120;
    //        params.minSpeed = 3.5/*4*//*15*/;
    //        params.skoCoef = 2.5;
    //        params.maxAngleBetwDirections = 18;
    //        params.corrCoef = 0.7;
    //        params.searchAreaSize = 50;
    rois.indentSearchRect = rois.trackSecondRect;
    rois.indentSearchRect.x = 10;
    rois.indentSearchRect.y = 10;
    if (currentSearchROI.empty())
    {
        currentSearchROI = rois.mainSearchRect;
    }
#endif

#ifdef DEBUG_BALL_REC
    if (currentState == WaitForHit || currentState == BallHit)
    {
        changeParamsSituation(WaitForHit);
    }
    else if (currentState == WaitForMove || currentState == BallMove)
    {
        changeParamsSituation(WaitForMove);
    }
#endif

    frames.current = frame;
    if (frames.previous.empty())
    {
        frames.current.copyTo(frames.previous);
    }
    else
    {
        for (auto& i : cutFromPrevious)
        {
            if (i.counter >= 2)
            {
                i.templateCut.copyTo(frames.previous(i.rect));
                for (auto it = corWindows.begin(); it != corWindows.end(); ++it)
                {
                    if ((it->arrows.first().first.x - i.center.x < 2 &&
                         it->arrows.first().first.y - i.center.y < 2)
                            || (it->arrows.first().second.x - i.center.x < 2 &&
                                it->arrows.first().second.y - i.center.y < 2))
                    {
                        it = corWindows.erase(it);
                    }
                }
            }
            i.confirmed = false;
        }
        cv::absdiff(frames.current, frames.previous, frames.substracted);
        if (previousUpdateCounter == previousUpdateMax)
        {
            previousUpdateCounter = 0;
            frames.background.copyTo(frames.previous);
            qDebug() << "BACKGROUND CHANGED";
            if (!cutFromBackGround.empty())
            {
                frames.current(cutFromBackGround).copyTo(frames.previous(cutFromBackGround));
            }
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

        if (    currentState == WaitForBall
                || currentState == ProbablyBall
                || currentState == WaitForHit
                || currentState == WaitForMove)
        {
            handleBallNotFound(time);
            updateROIClosestPlayer();
        }
        handleTrack(time);
        if (previousUpdateCounter == 1)
        {
            if (!mainBallMoves.forks.isEmpty()
                    && mainBallMoves.forks.last().times.last().first == time
                    && mainBallMoves.forks.last().times.last().second)
            {
                cutFromBackGround = mainBallMoves.forks.last().rect;
            }
            else
            {
                cutFromBackGround = Rect();
            }
        }
#ifdef DEBUG_BALL_REC
        rectangle(frames.current, currentSearchROI, CV_RGB(125,125,125), 3);
        rectangle(frames.current, rois.mainSearchRect, CV_RGB(255,0,0));
        rectangle(frames.current, rois.trackFirstRect, CV_RGB(255,0,0));
        rectangle(frames.current, rois.indentSearchRect, CV_RGB(255,0,0));
        pedTracker->drawROIs(frames.current);
        imshow("video", frames.current);
        waitKey(0);
#endif

        if (currentState == WaitForHit || currentState == WaitForMove)
        {
            ++waitForEvent;
            if (waitForEvent == maxWaitForEvent)
            {
                interceptPlayerId = QUuid();
                waitForEvent = 0;
                currentState = WaitForBall;
                changeParamsSituation(currentState);
                mainBallMoves.forks.clear();
                currentSearchROI = rois.mainSearchRect;
            }
        }
    }
    return currentState;
}

void BallRecognizerPP::handleBallNotFound(qint64 time)
{
    cv::Scalar m, stdv;
    frames.threshold = frames.substracted(currentSearchROI);
    cv::meanStdDev(frames.threshold, m, stdv);

    cv::threshold(frames.threshold, frames.threshold, m[0] + params.skoCoef * stdv[0], 255, cv::THRESH_BINARY);
    cv::Mat subImg = frames.threshold;

    medianBlur(subImg, subImg, 5);

    Mat canny_output;
    vector <vector<Point> > contours;
    Canny(subImg, canny_output, params.cannyThresMin, params.cannyThresMax, 3 );
    findContours( canny_output, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0) );

#ifdef DEBUG_BALL_REC
    imshow("window2", canny_output);
    waitKey(0);
#endif

    if (!contours.empty())
    {
        for (quint32 j = 0; j < contours.size(); ++j)
        {
            auto area = contourArea(contours[j]);
            auto mmnts =  moments( contours[j], true);
            auto circularity = circularityMetric( mmnts );
            if (area >= params.minArea && area <= params.maxArea
                    && circularity > params.circularityCoeff)
            {
                auto center = Point2f( mmnts.m10 / mmnts.m00 , mmnts.m01 / mmnts.m00 );

                CorDataRaw tmp;
                qint32 offset = sqrt(area);
                tmp.center = center;

                tmp.center.x  += currentSearchROI.x;
                tmp.center.y  += currentSearchROI.y;

                bool nonAprior = false;
                if (currentState == WaitForMove || currentState == BallMove)
                {
                    nonAprior = true;
                }

                if (pedTracker->intersectsAnyPedestrian(tmp.center, nonAprior).intercept)
                {
                    continue;
                }

                auto rect = boundingRect(contours[j]);
                auto cutRect = rect;
                cutRect.x += currentSearchROI.x;
                cutRect.y += currentSearchROI.y;
                rect.x += currentSearchROI.x - offset;
                rect.y += currentSearchROI.y - offset;
                rect.width += offset * 2;
                rect.height += offset * 2;
                tmp.rect = rect;
                checkRectangleSize(tmp.rect, frames.substracted.cols, frames.substracted.rows);

                tmp.objTemplates.push_back(Mat());
                frames.substracted(tmp.rect)
                        .copyTo(tmp.objTemplates.back());
                tmp.objGrayTemplates.push_back(Mat());
                frames.current(tmp.rect)
                        .copyTo(tmp.objGrayTemplates.back());
                tmp.arrows.append(qMakePair(tmp.center, tmp.center));
                tmp.times.append(qMakePair(time, true));
                corWindows.push_back(tmp);

                bool cutFind = false;
                for (auto& i : cutFromPrevious)
                {
                    if (abs(i.center.x - tmp.center.x) < 2
                            && abs(i.center.y - tmp.center.y) < 2)
                    {
                        ++i.counter;
                        cutFind = true;
                        i.confirmed = true;
                        frames.current(cutRect).copyTo(i.templateCut);
                        i.rect = cutRect;
                        break;
                    }
                }
                if (!cutFind)
                {
                    CutFromPrevious cut;
                    cut.center = tmp.center;
                    frames.current(cutRect).copyTo(cut.templateCut);
                    cut.rect = cutRect;
                    ++cut.counter;
                    cutFromPrevious.append(cut);
                }
            }
        }
    }
    qint32 k = 0;
    while (k < cutFromPrevious.size())
    {
        if (!cutFromPrevious[k].confirmed)
        {
            cutFromPrevious.remove(k);
        }
        else
        {
            ++k;
        }
    }
    if (!corWindows.isEmpty() && currentState == WaitForBall)
    {
        currentState = ProbablyBall;
    }
}

void BallRecognizerPP::joinForkedTrajectories()
{
    qint32 penultIndex = 2;
    auto  penult = mainBallMoves.forks.end() - penultIndex;
    auto lastTrajectory = &mainBallMoves.forks.last();
    while (!penult->times.back().second)
    {
        penult->times.removeLast();
        penult->arrows.removeLast();
    }

    penult->forked = false;
    penult->arrows.append(lastTrajectory->arrows);
    penult->times.append(lastTrajectory->times);
    penult->center = lastTrajectory->center;
    penult->currentSpeed = lastTrajectory->currentSpeed;
    penult->objGrayTemplates.append(lastTrajectory->objGrayTemplates);
    penult->objTemplates.append(lastTrajectory->objTemplates);
    penult->rect = lastTrajectory->rect;
    penult->closed = false;
    mainBallMoves.forks.removeLast();
}


Rect BallRecognizerPP::chooseTrackerSearchArea(CorDataRaw& corWin)
{
    qint32 leftBorder;
    qint32 upBorder;
    qint32 searchArea = 0;
    if (corWin.currentSpeed.x != 0 || corWin.currentSpeed.y != 0)
    {
        if (corWin.notFoundCount > 0)
        {
            searchArea = params.searchAreaSize / 1.5;
        }
        else
        {
            searchArea = params.searchAreaSize / 2;
        }

        Point2f p = Point2f(corWin.center.x + corWin.currentSpeed.x,
                            corWin.center.y + corWin.currentSpeed.y);
        leftBorder = p.x - searchArea;
        upBorder = p.y - searchArea;
    }
    else
    {
        searchArea = params.searchAreaSize;
        leftBorder = corWin.center.x  - params.searchAreaSize;
        upBorder = corWin.center.y  - params.searchAreaSize;
    }
    auto searchRect = Rect(leftBorder, upBorder,
                           2 * searchArea, 2 * searchArea);
    checkRectangleSize(searchRect, frames.substracted.cols, frames.substracted.rows);
    return searchRect;
}

double BallRecognizerPP::correlateBallTemplate(CorDataRaw& corWin, Mat& subFlip, Point& matchLoc, double& corrCoef, Rect& searchRect)
{
    qint32 result_cols =  subFlip.cols - corWin.objTemplates.back().cols + 1;
    qint32 result_rows = subFlip.rows - corWin.objTemplates.back().rows + 1;

    Mat result;
    result.create(result_rows, result_cols, CV_32FC1);
    matchTemplate(subFlip, corWin.objTemplates.back(), result, CV_TM_CCORR_NORMED);

    double minVal; double maxVal; Point minLoc; Point maxLoc;

    minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
    matchLoc = maxLoc;

    corrCoef = (corWin.dirAccordCount > 2 * ensureBallThreshold && corWin.notFoundCount == 0)
            ? params.corrCoef - 0.1 : params.corrCoef;
    qDebug() << maxVal << "correlation";
    if (maxVal < corrCoef && corWin.count >= acceptBallThreshold)
    {
        if (!corWin.notFoundCount)
        {
            searchRect.x = searchRect. x > 10 ? searchRect. x - 15 : 0;
            searchRect.y = searchRect.y > 10 ? searchRect.y - 15 : 0;
            searchRect.width = searchRect.x + searchRect.width + 30 > frames.substracted.cols ?
                        frames.substracted.cols - searchRect.x : searchRect.width + 30;
            searchRect.height = searchRect.y + searchRect.height + 30 > frames.substracted.rows ?
                        frames.substracted.cols - searchRect.x : searchRect.height + 30;
            frames.substracted(searchRect).copyTo(subFlip);
        }
        matchTemplate(subFlip, corWin.objTemplates[corWin.objTemplates.size() - 2], result, CV_TM_CCORR_NORMED);
        minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
        matchLoc = maxLoc;
    }
    //qDebug() << maxVal << "real correlation";
    imshow("window", subFlip);
    //waitKey(0);
    return maxVal;
}

Mat BallRecognizerPP::extractNewTemplate(CorDataRaw& corWin, Point matchLoc, Mat subFlip, Rect& rectTemplate)
{
    qint32 lb = matchLoc.x;
    qint32 ub = matchLoc.y;
    qint32 w = corWin.objTemplates.back().cols;
    qint32 h = corWin.objTemplates.back().rows;
    rectTemplate = Rect(lb, ub, w, h);
    checkRectangleSize(rectTemplate, subFlip.cols, subFlip.rows);
    Mat tmpTemplate;
    subFlip(rectTemplate).copyTo(tmpTemplate);
    return tmpTemplate;
}

qint32 BallRecognizerPP::clarifyBallCenter(CorDataRaw& corWin, Mat tmpTemplate, Point2f& center, Scalar& m, Scalar& stdv, bool& takeFromCenter, bool& takeOldDir)
{
    cv::meanStdDev(tmpTemplate, m, stdv);
    imshow("window3", tmpTemplate);
    qint32 countourCount = 0;
    if (corWin.isBall && (currentState != BallHit && currentState != WaitForHit))
    {
        Mat canny_output;
        vector <vector<Point> > contours;
        cv::threshold(tmpTemplate, tmpTemplate, m[0] + params.skoCoef * stdv[0], 255, cv::THRESH_BINARY);
        dilate(tmpTemplate, tmpTemplate, getStructuringElement(MORPH_RECT, Size(3, 3)));
        Canny(tmpTemplate, canny_output, params.cannyThresMin, params.cannyThresMax, 3);
        imshow("window4", canny_output);
        findContours(canny_output, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
        cv::Moments mmnts;
        countourCount  = contours.size();
        if (contours.size() == 1) // проверяем циркулярность найденного мяча
        {
            mmnts = moments( contours.front(), true);
            double circularity = circularityMetric(mmnts); // хранить среднюю площадь и проверять
            if (circularity > params.circularityCoeff)
            {
                takeFromCenter = true;
                center = Point2f( mmnts.m10 / mmnts.m00 , mmnts.m01 / mmnts.m00 );
            }
            else if (circularity < params.circularityCoeff / 2)
            {
                takeOldDir = true;
            }
        }
        else
        {
            bool anyCircle = false;
            double prevArea = -1;
            for (qint32 k = 0; k < contours.size(); ++k) // предпочитать те, что ближе к центру
            {
                mmnts = moments(contours[k], true);
                double circularity = circularityMetric(mmnts);
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
                takeOldDir = true;
            }
        }
    }
    return countourCount;
}

void BallRecognizerPP::increaseAccordCounter(CorDataRaw& corWin, BallTrackStatus& status)
{
    ++corWin.dirAccordCount;
    if (corWin.branch)
    {
        ++corWin.dirAccordCountBranch;
    }
    corWin.notFoundCount = 0;
    status = BallTrackStatus::Passed;
}

BallRecognizerPP::BallTrackStatus BallRecognizerPP::checkThresholds(CorDataRaw& corWin, double maxVal, Point2f newDir, bool takeOldDir, bool pedIntercept, qint64 time)
{
    BallTrackStatus status = Undefined;

    if (takeOldDir)
    {
        handleNotFound(corWin, time);
        status = BallTrackStatus::Missed;
    }
    else
    {
        double normOld = norm(corWin.currentSpeed);
        Point2f dirOldNorm = corWin.currentSpeed / normOld;
        double normNew = norm(newDir);
        Point2f dirNewNorm = newDir / normNew;
        double angle = acos(dirOldNorm.dot(dirNewNorm)) * radToDegrees;
        double relation = normOld > normNew ? normOld / normNew : normNew / normOld;
        qDebug() << corWin.arrows.first().first.x << "angle" << angle << "speed" << normNew  << "minSpeed" << params.minSpeed << "relation" << relation;
        if (angle < params.maxAngleBetwDirections
                && (normNew > params.minSpeed || corWin.dirAccordCount >= ensureBallThreshold)
                && relation < maxSpeedRelation)
        {
            increaseAccordCounter(corWin, status);
        }
        else if (currentState == BallHit
                 && corWin.dirAccordCount > ensureBallThreshold
                 && normNew < params.minSpeed
                 && relation < maxSpeedRelation)
        {
            increaseAccordCounter(corWin, status);
            changeParamsSituation(WaitForMove);
        }
        else if (corWin.justForked) // probably rebound from ground
        {
            increaseAccordCounter(corWin, status);
            corWin.firstForkedGround = true;
        }
        else if (mainBallMoves.forks.size() >= 2
                 && corWin.dirAccordCount > 3 * ensureBallThreshold
                 && corWin.count < acceptBallThreshold
                 && maxVal > 0.9
                 && !corWin.notFoundCount)
        {
            increaseAccordCounter(corWin, status);
        }
        else if (angle > params.maxAngleBetwDirections
                 && corWin.dirAccordCount >= ensureBallThreshold)
        {
            handleNotFound(corWin, time);

            if (pedIntercept)
            {
                status = BallTrackStatus::Missed;
            }
            else if (corWin.count >= acceptBallThreshold && !corWin.forked)
            {
                qDebug() << "FORKED" << angle;
                addFork(corWin, angle);
                status = BallTrackStatus::ForkTrajectory;
            }
            else
            {
                status = BallTrackStatus::Missed;
            }

        }
        else
        {
            ++corWin.notFoundCount;
            status = BallTrackStatus::Missed;
        }
    }

    if (corWin.justForked && status != ForkTrajectory)
    {
        corWin.justForked = false;
    }
    return status;
}



void BallRecognizerPP::updateNewState(auto lastTrajectory)
{
    switch (currentState)
    {
    case BallThrow:
        currentState = WaitForHit;
        maxWaitForEvent = maxWaitForHit;
        changeParamsSituation(currentState);
        currentSearchROI = rois.trackSecondRect;
        break;
    case BallHit:
    case BallMove:
    {
        currentState = WaitForMove;
        maxWaitForEvent = maxWaitForMove;
        changeParamsSituation(currentState);
        auto res = pedTracker->intersectsAnyPedestrian(lastTrajectory->center);
        if (res.intercept || res.distance < maxPlayerDistance)
        {
            Rect2d r = res.p->rUse;
            r.x -= r.width * 3;
            r.width += r.width * 6;
            r.y -= r.height;
            r.height += r.height * 2;
            currentSearchROI = r;
            checkRectangleSize(currentSearchROI, frames.current.cols, frames.current.rows);
            interceptPlayerId = res.p->id;
        }
        else
        {
            interceptPlayerId = QUuid();
            qint32 x, y, w, h;
            x = lastTrajectory->center.x - params.searchAreaSize * 3;
            y = lastTrajectory->center.y - params.searchAreaSize * 4;
            w = params.searchAreaSize * 6;
            h = params.searchAreaSize * 6;
            currentSearchROI = Rect(x, y, w, h);
            checkRectangleSize(currentSearchROI, frames.current.cols, frames.current.rows);
        }

        break;
    }
    default:
        break;
    }
}

void BallRecognizerPP::handleResults(BallTrackStatus status)
{
    if (!mainBallMoves.forks.isEmpty() )
    {
        auto lastTrajectory = &mainBallMoves.forks.last();
        bool branchConfirmed;
        if (status == ConfirmedAsBall  ||
                (branchConfirmed = (lastTrajectory->branch && lastTrajectory->dirAccordCountBranch == acceptBallThreshold)))
        {
            for (qint32 i = 0; i < lastTrajectory->arrows.size(); ++i)
            {
                emit ballRecognized(lastTrajectory->arrows[i].second, lastTrajectory->times[i].first);
            }
        }

        if (status == Passed)
        {
            if ((!lastTrajectory->branch && lastTrajectory->isBall)
                    || (lastTrajectory->branch && lastTrajectory->dirAccordCountBranch > acceptBallThreshold))
            {
                emit ballRecognized(lastTrajectory->arrows.last().second, lastTrajectory->times.last().first);
            }
        }
        if (branchConfirmed && lastTrajectory->forkedAngle < maxForkedAngle)
        {
            joinForkedTrajectories();
        }
    }
}

void BallRecognizerPP::handleTrack(qint64 time)
{
    qint32 penultIndex = 2;
    BallTrackStatus status;
    bool lastForkErased = false;
    if (mainBallMoves.forks.size() >= 2)
    {
        auto penult = mainBallMoves.forks.end() - penultIndex;
        if (!penult->closed)
        {
            status = handleBallFound(&*penult, time);
            if (status == Missed)
            {
                if (penult->notFoundCount >= acceptBallThreshold)
                {
                    penult->closed = true;
                }
            }
            else
            {
                penult->forked = false;
                mainBallMoves.forks.removeLast();
                lastForkErased = true;
            }
        }
    }
    if (!mainBallMoves.forks.isEmpty()
            && !mainBallMoves.forks.last().closed
            && !lastForkErased)
    {
        auto lastTrajectory = &mainBallMoves.forks.last();
        status = handleBallFound(lastTrajectory, time);
        if (status == Missed || status == MissedFromFrame)
        {
            if (status == MissedFromFrame)
            {
                emit ballOutOfFrame(dir);
            }
            if (lastTrajectory->notFoundCount >= acceptBallThreshold + 1)
            {
                auto penult = mainBallMoves.forks.end() - penultIndex;

                if (mainBallMoves.forks.size() == 1 || penult->closed)
                {
                    lastTrajectory->closed = true;
                    updateNewState(lastTrajectory);
                }
                else
                {
                    mainBallMoves.forks.removeLast();
                }
            }
        }
    }

    for (auto it = corWindows.begin(); it != corWindows.end(); ++it)
    {
        BallTrackStatus status = handleBallFound(&*it, time);
        if (status == ConfirmedAsBall)
        {
            corWindows.clear();
            break;
        }
        else if (status == MissedFromFrame)
        {
            it->forceDelete = true;
        }
    }
    auto it = corWindows.begin();
    while (it != corWindows.end())
    {
        if (it->notFoundCount == acceptBallThreshold || it->forceDelete)
        {
            it = corWindows.erase(it);
        }
        else
        {
            ++it;
        }
    }

    handleResults(status);
}


Point2f BallRecognizerPP::chooseCenterOfBall(qint32 contoursCount, bool takeFromCenter,
                                             Rect searchRect, CorDataRaw* corWin, Point2f center,
                                             Point matchLoc, Mat tmpTemplate)
{

    Point2f realCenter = Point2f ( matchLoc.x + searchRect.x + center.x,
                                   matchLoc.y + searchRect.y + center.y);
    Point2f corrCenter = Point2f (searchRect.x + (matchLoc.x + (tmpTemplate.cols / 2)) ,
                                  searchRect.y + (matchLoc.y + (tmpTemplate.rows / 2)));
    Point2f prevCenter = corWin->center;
    if (contoursCount == 1 && takeFromCenter)
    {
        Point2f newDir = realCenter - prevCenter;
        double normOld = norm(corWin->currentSpeed);
        Point2f dirOldNorm = corWin->currentSpeed / normOld;
        double normNew = norm(newDir);
        Point2f dirNewNorm = newDir / normNew;
        double realAngle = acos(dirOldNorm.dot(dirNewNorm)) * radToDegrees;

        newDir = corrCenter - prevCenter;
        normNew = norm(newDir);
        dirNewNorm = newDir / normNew;
        double corrAngle = acos(dirOldNorm.dot(dirNewNorm)) * radToDegrees;
        if (realAngle > params.maxAngleBetwDirections / 2 && (realAngle / corrAngle) > maxAngleRelation)
        {
            center = corrCenter;
        }
        else
        {
            center = realCenter;
        }
    }
    else if (takeFromCenter)
    {
        center = realCenter;
    }
    else
    {
        center = corrCenter;
    }

    return center;
}

void BallRecognizerPP::updateROIClosestPlayer()
{
    if (!interceptPlayerId.isNull())
    {
        bool ok;
        auto p = pedTracker->findPlayerById(interceptPlayerId, ok);
        if (ok)
        {
            Rect r = p->rUse;
            r.x -= r.width * 3;
            r.width += r.width * 6;
            r.y -= r.height;
            r.height += r.height * 2;
            currentSearchROI = r;
            checkRectangleSize(currentSearchROI, frames.current.cols, frames.current.rows);
        }
    }
}

BallRecognizerPP::BallTrackStatus BallRecognizerPP::handleBallFound(CorDataRaw* corWin, qint64 time)
{
    BallTrackStatus status = Passed;

    if (corWin->isNew)
    {
        corWin->isNew = false;
        return status;
    }

    Rect searchRect = chooseTrackerSearchArea(*corWin);
    Mat subFlip = frames.substracted(searchRect);

    Point matchLoc;
    double corrCoef;
    double maxVal = correlateBallTemplate(*corWin, subFlip, matchLoc, corrCoef, searchRect);

    if (maxVal >= corrCoef)
    {
        Rect rectTemplate;
        Mat tmpTemplate = extractNewTemplate(*corWin, matchLoc, subFlip, rectTemplate);

        bool takeFromCenter = false;
        bool takeOldDir = false;
        Point2f center;
        cv::Scalar m, stdv;
        qint32 contoursCount = clarifyBallCenter(*corWin, tmpTemplate, center, m, stdv, takeFromCenter, takeOldDir);

        Point2f testCenter = chooseCenterOfBall(contoursCount, takeFromCenter, searchRect, corWin, center, matchLoc, tmpTemplate);

        Point2f prevCenter = corWin->center;
        Point2f newDir = testCenter - prevCenter;

        auto result = pedTracker->intersectsAnyPedestrian(testCenter);



        if (result.intercept)
        {
            ++corWin->interceptPed;
            if (corWin->interceptPed == acceptBallThreshold && !corWin->isBall)
            {
                corWin->forceDelete = true;
                status = Missed;
                return status;
            }
            else if (corWin->interceptPed == ensureBallThreshold)
            {
                corWin->notFoundCount = acceptBallThreshold + 1;
                status = Missed;
                return status;
            }
        }

        if (corWin->count > 0)
        {
            BallTrackStatus status = checkThresholds(*corWin, maxVal, newDir, takeOldDir, result.intercept, time);
            switch (status)
            {
            case Missed:
                return status;
            case ForkTrajectory:
                corWin = &mainBallMoves.forks.last();
                break;
            default:
                break;
            }
        }

        corWin->center = testCenter;
        corWin->currentSpeed = newDir;
        corWin->arrows.append(qMakePair(prevCenter, corWin->center));
        corWin->times.append(qMakePair(time, true));

        if (corWin->dirAccordCount >= acceptBallThreshold)
        {
            checkNonApriorMeasures(*corWin);
            if (find_if(mainBallMoves.forks.begin(), mainBallMoves.forks.end(),
                               [corWin](auto& a){return a.id == corWin->id;}) == mainBallMoves.forks.end())
            {
                mainBallMoves.forks.append(*corWin);
                corWin = &mainBallMoves.forks.last();
            }

            if (currentState != BallThrow
                    && currentState != BallHit
                    && currentState != BallMove)
            {
                if (currentState == WaitForHit)
                {
                    currentState = BallHit;
                    waitForEvent = 0;
                }
                else if (currentState == WaitForMove)
                {
                    currentState = BallMove;
                    waitForEvent = 0;
                }
                else
                {
                    currentState = BallThrow;
                }
                corWin->isBall = true;
                status = ConfirmedAsBall;
            }
#ifdef DEBUG_BALL_REC
            rectangle(frames.current, Rect(corWin->center.x - 10, corWin->center.y - 10, 20, 20), CV_RGB(255, 255, 255));
            for (auto& ar : corWin->arrows)
            {
                line(frames.current, ar.first, ar.second, CV_RGB(255, 0, 0));
            }
#endif
            OutOfFrame dir;
            if (!checkInSearchArea(*corWin, dir)) // проверять только граничную зону
            {
                Q_ASSERT(status != ConfirmedAsBall);
                status = MissedFromFrame;
                return status;
            }
        }

        if (stdv[0] > params.minSkoOnTemplate)
        {
            Rect r;
            if (takeFromCenter)
            {
                r  = Rect(matchLoc.x + center.x - corWin->objTemplates.back().cols / 2,
                          matchLoc.y + center.y - corWin->objTemplates.back().rows / 2,
                          corWin->objTemplates.back().cols + 1,
                          corWin->objTemplates.back().rows + 1);

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
            }
            else
            {
                r = rectTemplate;
            }

            corWin->objTemplates.push_back(Mat());
            subFlip(r).copyTo(corWin->objTemplates.back());
            r.x = r.x + searchRect.x;
            r.y = r.y + searchRect.y;
            corWin->objGrayTemplates.push_back(Mat());
            frames.current(r).copyTo(corWin->objGrayTemplates.back());

            corWin->rect = Rect(Point(searchRect.x + matchLoc.x, searchRect.y + matchLoc.y),
                                Point(searchRect.x + matchLoc.x + corWin->objTemplates.back().cols,
                                      searchRect.y + matchLoc.y + corWin->objTemplates.back().rows));
        }
        else
        {
            corWin->forceDelete = true;
        }
        ++corWin->count;
    }
    else
    {
        handleNotFound(*corWin, time);
        status = Missed;
    }
    return status;
}


void BallRecognizerPP::handleNotFound(CorDataRaw& corWin, qint64 time)
{
    auto prevCenter = corWin.center;
    corWin.center.x += corWin.currentSpeed.x;
    corWin.center.y += corWin.currentSpeed.y;
    corWin.arrows.append(qMakePair(prevCenter, corWin.center));
    corWin.times.append(qMakePair(time, false));
    ++corWin.notFoundCount;
    corWin.previousWasNotFound = true;
}

void BallRecognizerPP::addFork(BallRecognizerPP::CorDataRaw& data, double forkedAngle)
{
    Q_ASSERT(&mainBallMoves.forks.last() == &data);
    CorDataRaw newFork;
    newFork.dirAccordCount = data.dirAccordCount < ensureBallThreshold ? ensureBallThreshold : data.dirAccordCount;
    newFork.isBall = true;
    newFork.objTemplates.append(data.objTemplates.back());
    newFork.forkState = currentState;
    newFork.justForked = true;
    newFork.branch = true;
    newFork.forkedAngle = forkedAngle;
    data.forked = true;


    mainBallMoves.forks.append(newFork);
}


void BallRecognizerPP::getOutOfFrameDirection(Point2f& prevCenter, OutOfFrame& dir)
{
    if (prevCenter.x < 100)
    {
        dir = OutOfFrame::Left;
    }
    else if (prevCenter.x > frames.current.cols - 100)
    {
        dir = OutOfFrame::Right;
    }
    else if (prevCenter.y < 100)
    {
        dir = OutOfFrame::Up;
    }
    else if (prevCenter.y > frames.current.rows - 100)
    {
        dir = OutOfFrame::Down;
    }
}

bool BallRecognizerPP::checkInSearchArea(CorDataRaw& corWin, OutOfFrame& dir)
{
    Point2f prevCenter = corWin.center;
    prevCenter.x += corWin.currentSpeed.x;
    prevCenter.y += corWin.currentSpeed.y;
    bool result = false;
    if (currentState == BallThrow)
    {
        if (rois.trackFirstRect.contains(Point2f(prevCenter.x, prevCenter.y)))
        {
            result = true;
        }
        else
        {
            ++corWin.notFoundCount;
            getOutOfFrameDirection(prevCenter, dir);
            //            if (corWin.isBall)
            //            {
            //                corWin.forceDelete = true;
            //                currentState = WaitForHit;
            //            }
        }
    }
    else if (currentState == BallHit)
    {
        if (rois.trackSecondRect.contains(Point2f(prevCenter.x, prevCenter.y)))
        {
            result = true;
        }
        else
        {
            ++corWin.notFoundCount;
            getOutOfFrameDirection(prevCenter, dir);
            //            if (corWin.isBall)
            //            {
            //                currentState = WaitForBall;
            //                corWin.forceDelete = true;

            //            }
        }
    }
    else if (currentState == BallMove)
    {
        if (rois.indentSearchRect.contains(Point2f(prevCenter.x, prevCenter.y)))
        {
            result = true;
        }
        else
        {
            getOutOfFrameDirection(prevCenter, dir);
            ++corWin.notFoundCount;
        }
    }

    return result;
}

void BallRecognizerPP::changeParamsSituation(State state)
{
//    if (!(state == currentState))
//    {
        if (state == WaitForBall)
        {
            params = initParams;
        }
        else if (state == WaitForHit)
        {
            params.circularityCoeff = 0.7;
            params.maxArea = initParams.maxArea * 2.5;
            params.searchAreaSize = initParams.searchAreaSize * 2.25;
            params.minSpeed = initParams.minSpeed - 5;
        }
        else if (state == WaitForMove)
        {
            params.minSpeed = initParams.minSpeed - 11.5;
            params.minArea = initParams.minArea * 0.3;
        }
    //}
}


void BallRecognizerPP::checkNonApriorMeasures(CorDataRaw& corWin)
{
    if (corWin.dirAccordCount == acceptBallThreshold + 1 && !corWin.forked)
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
            double angle = acos(dirFirst.dot(dirSecond)) * radToDegrees;
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


double BallRecognizerPP::circularityMetric(Moments& mmnts)
{
    return pow(mmnts.m00, 2) / (2 * M_PI * (mmnts.mu20 + mmnts.mu02)) ;
}
