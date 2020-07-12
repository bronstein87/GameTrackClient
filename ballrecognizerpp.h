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
#include <QElapsedTimer>
#include <QDir>
#include <QDateTime>
#include <QTextStream>
#include <mainstructs.h>
#include <QLinkedList>
#include <QUuid>
#include <pedestriantracker.h>
#include <opencvhelpfunction.h>
#include <proto/msg.internal.pb.h>
#include <proto/proto_helper.h>
#include <mathfunc.h>

#define DEBUG_BALL_REC

using namespace std;
using namespace cv;
using namespace gt::internal;
using namespace BOKZMath;

class BallRecognizerPP : public QObject
{
    Q_OBJECT

public:

    enum State
    {
        Invalid,
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

    struct BallMeta
    {
        BallMeta() {}
        BallMeta (qint64 _time, bool _valid, bool _rebound) :
            time (_time), valid(_valid), rebound(_rebound) {}
        qint64 time;
        bool valid;
        bool rebound = false;
    };

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
        QVector <BallMeta> meta;
        bool isBall = false;
        cv::Rect rect;
        bool forceDelete = false;
        bool previousWasNotFound = false;
        bool isNew = true;
        State forkState;
        bool closed = false; // закрыта ли данная траектория (т.е заменена ли новой веткой)
        bool forked = false; // существует ли форк для текущей ветки
        bool branch = false; // ветка ли данная траектория
        bool justForked = false; // была ли ветка образована на предыдущей итерации
        QUuid id = QUuid::createUuid();
        qint32 intersectPed = 0;
        double forkedAngle = 0;
    };



    explicit BallRecognizerPP(QObject *parent = nullptr);

    State recognize(Mat frame, qint64 time);

    void setInitialState(State state);

    void setPedestrianTracker (PedestrianTracker* tracker) {pedTracker = tracker;}

    void setCurrentSearchROI(cv::Rect r);

    void setROIs(msg::RecROIs* _rois);

    void setRecognizeParams(msg::RecognizeParameters* recParams);

    void analyzePreviousFrames(CorDataRaw& recData, const QVector <FrameInfo>& prev);

    void setDebug(bool debugFlag);

    void clear();

    void setBackGroundFrame(Mat frame);

signals:

    void errorOccured(const QString& text);

    void ballOutOfFrame(msg::OutOfFrame dir, const Point2f& lastPoint);

    void ballRecognized(msg::BallMeasure measure);

    void resultsReady(const State state, const CorDataRaw& results);

private:

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
        cv::Rect rect;
    };



    double circularityMetric(Moments& mmnts);

    void handleTrack(qint64 time);

    BallTrackStatus handleBallFound(CorDataRaw* corWin, qint64 time);

    void handleNotFound(CorDataRaw& corWin, qint64 time);

    void handleBallNotFound(qint64 time);

    void changeParamsSituation(BallRecognizerPP::State state);

    cv::Rect chooseTrackerSearchArea(CorDataRaw& corWin);

    double correlateBallTemplate(CorDataRaw& corWin, Mat& subFlip, Point& matchLoc, double& corrCoef, cv::Rect& searchRect);

    Mat extractNewTemplate(CorDataRaw& corWin, Point matchLoc, Mat subFlip, cv::Rect& rectTemplate);

    qint32 clarifyBallCenter(CorDataRaw& corWin, Mat tmpTemplate, Point2f& center, Scalar& m, Scalar& stdv, bool& takeFromCenter, bool& takeOldDir);

    bool checkInSearchArea(CorDataRaw& corWin, msg::OutOfFrame& dir);

    void addFork(CorDataRaw& data, double forkedAngle);

    void joinForkedTrajectories();

    void checkNonApriorMeasures(CorDataRaw& corWin);

    void getOutOfFrameDirection(Point2f& prevCenter, msg::OutOfFrame& dir);

    BallTrackStatus checkThresholds(CorDataRaw& corWin, double maxVal, Point2f newDir, bool takeOldDir, bool pedIntersect, qint64 time);

    void updateNewState(const CorDataRaw& lastTrajectory);

    void updateWaitState();

    Point2f chooseCenterOfBall(qint32 contoursCount, bool takeFromCenter, cv::Rect searchRect,
                               CorDataRaw* corWin, Point2f center, Point matchLoc, Mat tmpTemplate);

    void updateROIClosestPlayer();

    void increaseAccordCounter(CorDataRaw& corWin, BallTrackStatus& status);

    void handleResults(BallTrackStatus status, State previousState);

    msg::BallMeasure fillBallMeasure(const Point2f& position, const BallMeta& meta, msg::BallEvent ballEvent);

    PedestrianTracker* pedTracker;

    State currentState = WaitForBall;

    cv::Rect currentSearchROI;

    msg::RecROIs* rois;

    msg::RecognizeParameters params;

    msg::RecognizeParameters* initParams;

    QVector <CutFromPrevious> cutFromPrevious;

    QLinkedList <CorDataRaw> corWindows;

    cv::Rect cutFromBackGround = cv::Rect();

    msg::OutOfFrame outOfFrameDir;

    CorData mainBallMoves;

    Frames frames;

    QUuid intersectPlayerId;

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

    constexpr const static qint32 outOfFrameBorder = 100;





};

#endif // BALLRECOGNIZERPP_H
