#include "battracker.h"

BatTracker::BatTracker(qint32 width, qint32 height, QObject *parent) : DetectNetBase(width, height, parent)
{

}

BatTracker::BatTrackerStatus BatTracker::handle(Mat frame, qint64 time)
{
    if (status == Invalid)
    {
        status = WaitForBall;
    }
    if (status == WaitForBall)
    {
        cvtColor(frame, frame, COLOR_BGR2GRAY);
        auto ballRecState = ballRecognizer->recognize(frame, time);
        if (ballRecState == BallRecognizerPP::BallThrow)
        {
            ballRecognizer->clear();
            status = BallFound;
        }
        else
        {
            status = WaitForBall;
        }
    }
    else if (status == BallFound || status == RecognizeBat)
    {
        status = RecognizeBat;
        Mat inputImg;
        cv::Rect cutRect;
        if (previousRecognized)
        {
            Point center = Point(recognizedROIs.back().first.x + recognizedROIs.back().first.width / 2,
                                 recognizedROIs.back().first.y + recognizedROIs.back().first.height/ 2);
            cutRect = cv::Rect(center.x - maxImageSize / 2,
                           center.y - maxImageSize / 2,
                           maxImageSize, maxImageSize);
            qDebug() << cutRect.x << cutRect.y << cutRect.width << cutRect.height;
            checkRectangleSize(cutRect, 1920, 1080);
            inputImg = frame(cutRect);
        }
        else
        {
            inputImg = frame;
        }

        //imshow("video", inputImg);
        //qDebug() << "prev";
        //waitKey(0);
        readFrameInGPUMemory(inputImg);
        detectNet::Detection* detections = nullptr;
        net->interceptCoeff = 0.5;
        const qint32 numDetections = net->Detect(imgCUDA, inputImg.cols, inputImg.rows, &detections, detectNet::OverlayFlags::OVERLAY_NONE);
        QVector < QPair <double, Rect2d> > v;

        if (numDetections > 0)
        {
            for( qint32 n = 0; n < numDetections; ++n )
            {
                v.append(qMakePair(detections[n].Confidence, Rect2d(detections[n].Left, detections[n].Top, detections[n].Width(), detections[n].Height())));
                if (!cutRect.empty())
                {
                    v.back().second.x = v.back().second.x + cutRect.x;
                    v.back().second.y = v.back().second.y + cutRect.y;
                }
            }
            if (v.size() > 1)
            {
                std::sort(v.begin(), v.end(), [](auto& f, auto& s){return f.first > s.first;});
            }

            auto chosenROI = v.first();
            if (!recognizedROIs.isEmpty() && v.size() > 1)
            {
                double iou = 0;
                for (auto& i : v)
                {
                    auto res = calculateIOU(recognizedROIs.back().first, i.second);
                    if (iou < res.iou)
                    {
                        iou = res.iou;
                        chosenROI = i;
                    }
                }
            }
            float* data = nullptr;

            data = net->heatMaps.heatMap.front().get();
            for (qint32 i = 0; i < net->heatMaps.h; ++i)
            {
                QString line;
                for (qint32 j = 0; j < net->heatMaps.w; ++j)
                {
                    data[j * net->heatMaps.h + i] *= 255;
                    line.append(QString::number((int)(data[j * net->heatMaps.h + i])) + "  ");
                }
                qDebug() << line;
            }


            rectangle(frame, chosenROI.second, CV_RGB(255, 255, 255), 2);
            imshow("video", frame);
            //imshow("video", mserFrame);
            qDebug() << "res";
            waitKey(0);



            recognizedROIs.append(qMakePair(chosenROI.second, Mat(Size(net->heatMaps.h, net->heatMaps.w), CV_32FC1, net->heatMaps.heatMap.front().get())));
            //previousRecognized = true;
        }
        else
        {
            previousRecognized = false;
        }

        ++recognizeBatCount;
        if (recognizeBatCount == recognizeBatFor)
        {
            status = WaitForBall;
        }
    }
    qDebug() << "exit";
    return status;
}

void BatTracker::setBallRecognizer(BallRecognizerPP *_ballRecognizer)
{
    ballRecognizer = _ballRecognizer;
}
