#ifndef MAINSTRUCTS
#define MAINSTRUCTS
#include <QtGlobal>
#include <opencv2/core.hpp>

using namespace cv;


struct FrameInfo
{
    FrameInfo (Mat _frame, qint64 _time) :time(_time) {_frame.copyTo(frame);}
    FrameInfo(){}
    cv::Mat frame;
    quint64 time = 0;
    quint64 computerTime = 0;
    qint32 memoryId = -1;
    bool mainFrame = false;
    bool handled = false;
    bool sent = false;
};




#endif // MAINSTRUCTS

