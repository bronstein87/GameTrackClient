#ifndef BALLRECOGNIZERPP_H
#define BALLRECOGNIZERPP_H
#define _USE_MATH_DEFINES
#include <cmath>
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
#include <QLinkedList>
#include <QUuid>
#include <pedestriantracker.h>
#include <opencvhelpfunction.h>
#define DEBUG_BALL_REC

using namespace std;
using namespace cv;
class BallRecognizerPP : public QObject
{
    Q_OBJECT

public:

    enum State
    {
        WaitForBall,
        WaitForHit,
        WaitForMove,
        ProbablyBall,
        BallThrow,
        BallHit,
        BallMove
    };


    enum BallTrackStatus
    {
        Undefined,
        ConfirmedAsBall,
        Passed,
        ForkTrajectory,
        Missed,
        MissedFromFrame
    };



    explicit BallRecognizerPP(QObject *parent = nullptr);

    State recognize(Mat frame, qint64 time);

    void setInitialState(State state)
    {
        currentState = state;
        changeParamsSituation(state);
    }

    void setPedestrianTracker (QSharedPointer <PedestrianTracker> tracker) {pedTracker = tracker;}

    void setCurrentSearchROI(Rect r) { currentSearchROI = r;}

    void setMainROI(Rect r) {rois.mainSearchRect = r;}

    void setFirstROI(Rect r) {rois.trackFirstRect = r;}

    void setSecondROI(Rect r) {rois.trackSecondRect = r;}

    void setIndentROI(Rect r){rois.indentSearchRect = r;}

    void setBackGroundFrame(Mat frame)
    {
        previousUpdateCounter = 0;
        frame.copyTo(frames.previous);
    }



signals:

    void errorOccured(const QString& text);

    void ballOutOfFrame(OutOfFrame dir);

    void ballRecognized(Point2f coords, qint64 time);


private:

    struct CorDataRaw
    {
        CorDataRaw() : currentSpeed(0, 0) {}
        Point2f center;
        QVector <Mat> objTemplates;
        QVector <Mat> objGrayTemplates;
        qint32 count = 0;
        qint32 notFoundCount = 0;
        Point2f currentSpeed;
        qint32 dirAccordCount = 0;
        qint32 dirAccordCountBranch = 0;
        QVector <QPair <Point2f, Point2f>> arrows;
        QVector <QPair<qint64, bool>> times;
        bool isBall = false;
        Rect rect;
        bool forceDelete = false;
        bool previousWasNotFound = false;
        bool isNew = true;
        State forkState;
        bool closed = false;
        bool forked = false;
        bool branch = false;
        bool justForked = false;
        bool firstForkedGround = false;
        QUuid id = QUuid::createUuid();
        qint32 interceptPed = 0;
        double forkedAngle;
    };

    struct CorData
    {
        QLinkedList <CorDataRaw> forks;
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

    struct CutFromPrevious
    {
        Mat templateCut;
        Point2f center;
        qint32 counter = 0;
        bool confirmed = true;
        Rect rect;
    };



    double circularityMetric(Moments& mmnts);

    void handleBallNotFound(qint64 time);

    void joinForkedTrajectories();

    void handleTrack(qint64 time);

    BallRecognizerPP::BallTrackStatus handleBallFound(CorDataRaw* corWin, qint64 time);

    void changeParamsSituation(BallRecognizerPP::State state);

    Rect chooseTrackerSearchArea(CorDataRaw& corWin);

    double correlateBallTemplate(CorDataRaw& corWin, Mat& subFlip, Point& matchLoc, double& corrCoef, Rect& searchRect);

    Mat extractNewTemplate(CorDataRaw& corWin, Point matchLoc, Mat subFlip, Rect& rectTemplate);

    qint32 clarifyBallCenter(CorDataRaw& corWin, Mat tmpTemplate, Point2f& center, Scalar& m, Scalar& stdv, bool& takeFromCenter, bool& takeOldDir);

    bool checkInSearchArea(CorDataRaw& corWin, OutOfFrame& dir);

    void handleNotFound(CorDataRaw& corWin, qint64 time);

    void addFork(CorDataRaw& data, double forkedAngle);

    void checkNonApriorMeasures(CorDataRaw& corWin);

    void getOutOfFrameDirection(Point2f& prevCenter, OutOfFrame& dir);

    BallRecognizerPP::BallTrackStatus checkThresholds(CorDataRaw& corWin, double maxVal, Point2f newDir, bool takeOldDir, bool pedIntercept, qint64 time);

    void updateNewState(auto lastTrajectory);

    Point2f chooseCenterOfBall(qint32 contoursCount, bool takeFromCenter, Rect searchRect,
                               CorDataRaw* corWin, Point2f center, Point matchLoc, Mat tmpTemplate);

    void updateROIClosestPlayer();

    void increaseAccordCounter(CorDataRaw& corWin, BallTrackStatus &status);

    void handleResults(BallTrackStatus status);

    QSharedPointer <PedestrianTracker> pedTracker;

    State currentState = WaitForBall;

    Rect currentSearchROI;

    RecROIs rois;

    QVector <CutFromPrevious> cutFromPrevious;

    QLinkedList <CorDataRaw> corWindows;

    Rect cutFromBackGround = Rect();

    OutOfFrame dir;

    CorData mainBallMoves;

    Frames frames;

    RecognizeParameters params;

    RecognizeParameters initParams;

    QUuid interceptPlayerId;

    static constexpr const qint32 maxWaitForMove = 200;

    static constexpr const qint32 maxWaitForHit = 20;

    qint32 maxWaitForEvent = 0;

    qint32 waitForEvent = 0;

    qint32 previousUpdateCounter = 0;

    double previousCorr = -1;

    constexpr const static double radToDegrees = 57.295779513;

    constexpr const static qint32 previousUpdateMax = 8;

    constexpr const static qint32 acceptBallThreshold = 3;

    constexpr const static qint32 ensureBallThreshold = 5;

    constexpr const static qint32 maxSpeedRelation = 2.2;

    constexpr const static qint32 maxAngleRelation = 2;

    constexpr const static qint32 maxPlayerDistance = 200;

     constexpr const static qint32 maxForkedAngle = 28;


};

#endif // BALLRECOGNIZERPP_H
