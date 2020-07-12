#ifndef BALLRECOGNIZER_H
#define BALLRECOGNIZER_H

#include <QObject>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <QDebug>
#include <vector>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <QElapsedTimer>
#include <QDir>
#include <QDateTime>
#include <QTextStream>
#include <mainstructs.h>
#include <mathfunc.h>
using namespace BOKZMath;


using namespace  cv;
using namespace  std;

class CorData;


class BallRecognizer : public QObject
{
    Q_OBJECT

    struct CorData
    {
        CorData() : dir(10, 10) {}
        Point2f center;
        QVector <Mat> objTemplates;
        QVector <Mat> objGrayTemplates;
        qint32 count = 0;
        qint32 notFoundCount = 0;
        Point2f dir;
        qint32 dirAccordCount = 0;
        QVector <QPair <Point2f, Point2f>> arrows;
        QVector <QPair<qint64, bool>> times;
        bool isBall = false;
        Rect rect;
        bool forceDelete = false;
        Point2f previousSpeed = Point2f(0, 0);
        bool previousWasNotFound = false;
        bool isNew = true;
    };


    struct Frames
    {
        Mat current;
        Mat previous;
        Mat substracted;
        Mat interested;
        Mat background;
        Mat threshold;
    };



    struct RecResults
    {
        CorData corWindowsThrow;
        CorData corWindowsHit;
    };

public:

    enum State
    {
        WaitForBall,
        WaitForHit,
        ProbablyBall,
        BallThrow,
        BallHit
    };


    explicit BallRecognizer(QObject *parent = 0);

    State recognize(Mat frame, qint64 time);

    void setROIs(RecROIs& r) {rois = r;}

    RecROIs getROIs() {return rois;}

    void setRecognizeParams(RecognizeParameters & p) {params = p;}

    void setMainROI(Rect r) {rois.mainSearchRect = r;}

    void setFirstROI(Rect r) {rois.trackFirstRect = r;}

    void setSecondROI(Rect r) {rois.trackSecondRect = r;}

    void setMainHitROI(Rect r){rois.mainHitSearchRect = r;}

    const RecognizeParameters& getRecognizeParameters() {return params;}

    void setCorrCoef(double value) {params.corrCoef = value;}

    void setSkoCoef (double value) {params.skoCoef = value;}

    void setSearchAreaSize(qint32 value) {params.searchAreaSize = value;}

    void setMinSkoOnTemplate(double value) {params.minSkoOnTemplate = value;}

    void setMaxAngleBetwDirections(double value) {params.maxAngleBetwDirections = value;}

    void setMinSpeed(double value) {params.minSpeed = value;}

    void setMinCanny(qint32 value) {params.cannyThresMin = value;}

    void setMaxCanny(qint32 value) {params.cannyThresMax = value;}

    void setMinArea(qint32 value) {params.minArea = value;}

    void setMaxArea(qint32 value) {params.maxArea = value;}

    void setCircularityCoef(double value) {params.circularityCoeff = value;}

    void setThrowDirX(double dirX) {params.dirX = dirX;}

    void setThrowDirY(double dirY) {params.dirY = dirY;}

    double getThrowDirX() {return params.dirX;}

    double getThrowDirY() {return params.dirY;}

    void setDebugMessage(const QString& str);

    bool setDebug(bool flag);

    bool getDebug() {return debug;}

    RecResults getResults() {return results;}

    void analyzePreviousNextFrames(CorData& recData, const QVector<FrameTime>& prev, const QVector<FrameTime>& next);

    void debugPlotFile(const QString& filename);

    void clear()
    {
        currentState = WaitForBall;
        corWindows.clear();
        results = RecResults();
        frames = Frames();
        previousUpdateCounter = 0;
    }

    void setBackGroundFrame(Mat frame);



signals:

    void errorOccured(const QString& text);

    void resultsReady(BallRecognizer::State state);

private:

    void checkNonApriorMeasures(CorData& corWin);

    bool checkInSearchArea(CorData& corWin);

    void removeUnconfirmedOrLost();

    void handleBallNotFound(qint64 time);

    void handleBallFound(qint64 time);

    double circularityMetric(Moments& mmnts);

    void handleNotFound(CorData& corWin, qint64 time);

    void saveResults(CorData& corWin);

    void changeParamsForHit(bool back = false);

    RecROIs rois;

    State currentState = WaitForBall;

    QVector <CorData> corWindows;

    Frames frames;

    RecognizeParameters params;

    RecResults results;

    static constexpr const qint32 maxWaitForHitCount = 20;

    qint32 waitForHitCount = 0;

    double radToDegrees = 57.295779513;

    bool debug = false;

    QTextStream debugStream;

    QString debugDirName;

    QString tickDt;

    QFile plotFile;

    QFile telemetryFile;

    QVector <Point2f> v;

    QVector <Point2f> vinit;

    QVector <Point2f> vold;

    QVector <Point2f> voldinit;

    qint32 previousUpdateCounter = 0;

    constexpr const static qint32 previousUpdateMax = 8;

    constexpr const static qint32 acceptBallThreshold = 3;

   
};

Q_DECLARE_METATYPE(BallRecognizer::State)
#endif // BALLRECOGNIZER_H
