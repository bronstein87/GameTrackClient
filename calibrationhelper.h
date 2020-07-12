#ifndef CALIBRATIONHELPER_H
#define CALIBRATIONHELPER_H

#include <QObject>
#include <QFile>
#include <QTextStream>
#include <calibration.h>
#include <QXmlStreamReader>
#include <QDateTime>
#include <QString>
#include <proto/msg.internal.pb.h>
#include <QMap>

using namespace  gt::internal::msg;


class CalibrationHelper
{

private:

    CalibrationHelper();
    QMap <QString, Calibration::ExteriorOr> eos;
    QMap <QString, Calibration::SpacecraftPlatform::CAMERA::CameraParams> cameras;
    QString calibrateDir = "calibrate/";
    QString currentCamera;


public:

    bool calculatePointOnGround(Calibration::Position2D& XYpix, Calibration::RayAndPoint& RayPoint, const QString &name = QString());

    void setEO(const QString& name, Calibration::ExteriorOr& _eo) {eos.insert(name, _eo);}

    Calibration::ExteriorOr getEO(const QString& name)
    {QString cam = name.isEmpty() ? currentCamera : name; return eos[cam];}

    void setCameraParams(const QString& name, Calibration::SpacecraftPlatform::CAMERA::CameraParams& _camera) {cameras.insert(name, _camera);}

    Calibration::SpacecraftPlatform::CAMERA::CameraParams getCameraParams(const QString& name)
    {QString cam = name.isEmpty() ? currentCamera : name; return cameras[cam];}

    void setCurrentCamera(const QString& camera) {currentCamera = camera;}

    static CalibrationHelper& instance();

    void convertToProto(Calibration::ExteriorOr& EO,
                               Calibration::SpacecraftPlatform::CAMERA::CameraParams& Camera,
                               CalibrationParameters& parameters);

    void convertFromProto(const CalibrationParameters &parameters,
                                 Calibration::SpacecraftPlatform::CAMERA::CameraParams& Camera,
                                 Calibration::ExteriorOr& EO);


    void readCurrentCalibrationParameters(const QString& id, const QString& path, Calibration::ExteriorOr& EO,
                                                 Calibration::SpacecraftPlatform::CAMERA::CameraParams& Camera,
                                                 bool current, qint32 w = 1920, qint32 h = 1080);


};

#endif // CalibrationHelper_H
