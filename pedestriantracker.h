#ifndef PEDESTRIANTRACKER_H
#define PEDESTRIANTRACKER_H

#include <QObject>
#include <QSharedPointer>
#include <QMutex>
#include <QVector>
#include <QLinkedList>
#include <QPair>
#include <QUuid>
#include <QMap>
#include <mainstructs.h>
#include <algorithm>
#include <detectnetbase.h>
#include <calibrationhelper.h>

using namespace cv;
using namespace std;




class PedestrianTracker : public DetectNetBase
{
    Q_OBJECT
public:


    enum class TeamRole
    {
        Undefined = 0,
        Batter = 1,
        Pitcher = 2,
        Catcher = 4,
        FirstBase = 8,
        SecondBase = 16,
        ThirdBase = 32,
        FirstDefence = 64,
        SecondDefence = 128,
        ThirdDefence = 256,
        ShortStop = 512,
        LeftFielder = 1024,
        RightFielder = 2048,
        CenterFielder = 4096
    };

    enum PlayerInterceptState
    {
        NoIntercept,
        Lost,
        OutOfFrameLost,
        ProbablyIntercept,
        DontMoveClose,
        DontMoveFar,
        MoveClose,
        MoveFar
    };

    enum class PlayerPosition
    {
        Home,
        FirstBase,
        SecondBase,
        ThirdBase,
        LeftField,
        CenterField,
        RightField
    };

    struct Player
    {
        Player() : id(QUuid::createUuid()) {}
        QUuid id;
        bool canComeBack = false;
        bool updated = false;
        qint32 trackNotFoundCount = 0;
        qint32 detNotFoundCount = 0;
        qint32 confirmed = 0;
        Rect2d rDet;
        Rect2d rTrack;
        Rect2d rUse;
        bool rTrackCropped = false;
        Ptr<TrackerMOSSE> tracker;
        double lastConfidence = 0;
        double lastTrackConfidence = -1.;
        PlayerInterceptState interceptState = NoIntercept;
        Point2f speed;
        qint32 speedUpdateCounter = 0;
        Rect2d prevR;
        Rect2d predictedR;
        Point2f lastSpeed;
        QPair <Mat, Mat> origHist;
        bool alreadyInited = false;
        QVector <QPair<quint64, Rect2d>> history;
        PlayerPosition position;
    };

    struct IOUResult
    {
        double iou;
        double partR1;
        double partR2;
    };


    using PPlayer = QSharedPointer<Player>;
    using LostPlayersArea = QPair <Rect2d, QVector <PPlayer>>;

    struct InterceptCheckResult
    {
        bool intercept = false;
        PPlayer p;
        double distance = 1000;
        TeamRole role = TeamRole::Undefined;
    };


    enum TrackMode
    {
        Teams,
        Players
    };

    enum State
    {
        Detect,
        Track,
    };

    explicit PedestrianTracker(qint32 width = 1920, qint32 height = 1080, QObject *parent = nullptr);

    void track(Mat _inputImg, quint64 time);

    void drawROIs(Mat drawImg);

    void setTrackZone(Rect2d r) {trackZone = r;}

    void setCameraPosition(CameraPosition p) {camPosition = p;}

    void setMode(TrackMode m) {mode = m;}

    void setDetectDuration(qint32 d) {detectDuration = d;}

    void setTrackDuration (qint32 d) {trackDuration = d;}

    void setState(State s) {state = s;}

    InterceptCheckResult intersectsAnyPedestrian(Point2f position, bool nonAprior = false);

    PPlayer findPlayerById(QUuid id, bool &ok);

    void clear();

    void setDebugBuffer (QLinkedList <FrameTime>* _buffer){buffer = _buffer;}


signals:

    void outOfFrame(OutOfFrame dir);

private:

    QVector <LostPlayersArea> checkPlayersIntercept();

    Point3f evaluate3dPosition(Rect r, bool& ok);

    IOUResult calculateIOU(Rect2d r1, Rect2d r2);

    QPair<Point2f, bool> checkIntersect(Player& p, Point2f& position, bool nonAprior = false);

    PedestrianTracker::InterceptCheckResult handleInterceptCheck(Point2f &position, PPlayer player,
                                                                 QVector <QPair <double, PPlayer>>& distance, bool nonAprior = false);

    void assignROIs(const qint32 numDetections, detectNet::Detection* detections);

    void finishInitialization();

    void trackPlayerInternal(Mat inputImg, PPlayer player, Mat grayResized, bool &erased);

    QPair<Mat, Mat> calculateBodyHueHist(Mat bodyImg, Rect2d r);

    void resolveConflict(PPlayer p1, PPlayer p2, Rect2d foundPlayer);

    void initTracker(Mat grayResized, PPlayer player);

    void initTrackerFirstly(Mat grayResized, PPlayer player);

    void savePlayerHistory(PPlayer p);


    qint32 detectCounter = 0;
    qint32 trackCounter = 0;
    double trackerThreshold = 3.5;
    double minIOUIntercept = 0.2;
    double minIntercept = 0.25;
    double trackConfidenceUpdateCoeff = 0.7;

    CameraPosition camPosition;
    Rect2d trackZone;
    qint32 detectDuration = 8;
    qint32 trackDuration = 20;
    QMap <TeamRole, PPlayer> attackInfo;
    QMap <TeamRole, PPlayer> defenceInfo;

    QMap <TeamRole, QVector <PPlayer> > unconfirmedPlayers;
    QLinkedList <PPlayer> nonStructuredPlayers;
    QVector <LostPlayersArea> interceptPlayers;
    QVector <PPlayer> outOfFramePlayers;
    TrackMode mode = Players;
    State state = Detect;
    QMutex mutex;
    Mat inputImg;
    QLinkedList <FrameTime>* buffer;
    quint64 lastTime;


    constexpr const static double resampleCoeff = 0.75;
    constexpr const static qint32 heightThreshold = 100;
    constexpr const static qint32 speedUpdateMax = 5;
    constexpr const static qint32 outOfFrameBorder = 10;
    constexpr const static double confidenceThreshold = 1.;
    constexpr const static double trackerHighConfidence = 4.5;
    constexpr const static double bodyCropCoeff = 0.6;
    constexpr const static double expandBodyROICoeff = 0.2;
    constexpr const static double maxTrackNotFound = 10;

};

//Q_DECLARE_OPERATORS_FOR_FLAGS(PedestrianTracker::TeamRoles)

#endif // PEDESTRIANTRACKER_H
