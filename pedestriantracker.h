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
#include <proto/msg.internal.pb.h>

using namespace cv;
using namespace std;
using namespace gt::internal;




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

    enum PlayerIntersectState
    {
        NoIntersect,
        Lost,
        OutOfFrameLost,
        ProbablyIntersect,
        DontMoveClose,
        DontMoveFar,
        MoveClose,
        MoveFar
    };

    enum class GamePositions
    {
        Invalid,
        Home,
        LeftSquare,
        RightSquare,
        Umpire,
        FirstBase,
        SecondBase,
        ThirdBase,
        LeftField,
        CenterField,
        RightField,
        ShortStop
    };

    struct MovePosition
    {
        Point2f onCam;
        Point3f onSpace;
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
        PlayerIntersectState intersectState = NoIntersect;
        Point2f speed;
        qint32 speedUpdateCounter = 0;
        Rect2d prevR;
        Rect2d predictedR;
        Point2f lastSpeed;
        QPair <Mat, Mat> origHist;
        bool alreadyInited = false;
        QPair<quint64, Rect2d> history;
        GamePositions position = GamePositions::Invalid;
        QVector <QPair <quint64, MovePosition>> moves;
        cv::Scalar color;
    };




    using PPlayer = QSharedPointer<Player>;
    using LostPlayersArea = QPair <Rect2d, QVector <PPlayer>>;

    struct IntersectCheckResult
    {
        bool intersect = false;
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

    void setMode(TrackMode m) {mode = m;}

    void setDetectDuration(qint32 d) {detectDuration = d;}

    void setTrackDuration (qint32 d) {trackDuration = d;}

    void setState(State s) {state = s;}

    IntersectCheckResult intersectsAnyPedestrian(Point2f position, bool nonAprior = false);

    PPlayer findPlayerById(QUuid id, bool &ok);

    void clear();

    void setDebugBuffer (QLinkedList <FrameInfo>* _buffer){buffer = _buffer;}

    QLinkedList <PPlayer>& getPlayersInfo() {return nonStructuredPlayers;}


signals:

    void outOfFrame(msg::OutOfFrame dir);

private:

    QVector <LostPlayersArea> checkPlayersIntersect();

    MovePosition evaluate3dPosition(cv::Rect r, bool& ok);

    QPair<Point2f, bool> checkIntersect(Player& p, Point2f& position, bool nonAprior = false);

    PedestrianTracker::IntersectCheckResult handleIntersectCheck(Point2f &position, PPlayer player,
                                                                 QVector <QPair <double, PPlayer>>& distance, bool nonAprior = false);

    void assignROIs(const qint32 numDetections, detectNet::Detection* detections);

    void finishInitialization();

    void trackPlayerInternal(Mat inputImg, PPlayer player, Mat grayResized, bool &erased);

    QPair<Mat, Mat> calculateBodyHueHist(Mat bodyImg, Rect2d r);

    void resolveConflict(PPlayer p1, PPlayer p2, Rect2d foundPlayer);

    void initTracker(Mat grayResized, PPlayer player);

    void initTrackerFirstly(Mat grayResized, PPlayer player);

    void savePlayerHistory(PPlayer p);

    void assignPosition(PPlayer player, bool &changed);


    qint32 detectCounter = 0;
    qint32 trackCounter = 0;
    double trackerThreshold = 3.5;
    double minIOUIntersect = 0.2;
    double minIntersect = 0.25;
    double trackConfidenceUpdateCoeff = 0.7;

    Rect2d trackZone;
    qint32 detectDuration = 8;
    qint32 trackDuration = 20;
    QMap <TeamRole, PPlayer> attackInfo;
    QMap <TeamRole, PPlayer> defenceInfo;

    QMap <TeamRole, QVector <PPlayer> > unconfirmedPlayers;
    QLinkedList <PPlayer> nonStructuredPlayers;
    QVector <LostPlayersArea> intersectPlayers;
    QVector <PPlayer> outOfFramePlayers;
    TrackMode mode = Players;
    State state = Detect;
    QMutex mutex;
    Mat inputImg;
    QLinkedList <FrameInfo>* buffer;
    quint64 lastTime;
    QMap <GamePositions, QPair <QString, Point3f>> positionCoordinates;


    constexpr const static double resampleCoeff = 0.75;
    constexpr const static qint32 heightThreshold = 100;
    constexpr const static qint32 speedUpdateMax = 5;
    constexpr const static qint32 outOfFrameBorder = 10;
    constexpr const static double confidenceThreshold = 1.;
    constexpr const static double trackerHighConfidence = 4.5;
    constexpr const static double bodyCropCoeff = 0.6;
    constexpr const static double bodyWidthHeightProportion = 1.5;
    constexpr const static double expandBodyROICoeff = 0.25;
    constexpr const static double maxTrackNotFound = 10;
    constexpr const static qint32 tryToTrack = 3;
    constexpr const static double speedThreshold = 10;

};

//Q_DECLARE_OPERATORS_FOR_FLAGS(PedestrianTracker::TeamRoles)

#endif // PEDESTRIANTRACKER_H
