#ifndef CALIBRATIONHELPER_H
#define CALIBRATIONHELPER_H

#include <QObject>
#include <QFile>
#include <QTextStream>
#include <calibration.h>
#include <QXmlStreamReader>
#include <QDateTime>
#include <QString>


class CalibrationHelper
{

private:
    CalibrationHelper();
    Calibration::ExteriorOr EO;
    Calibration::SpacecraftPlatform::CAMERA::CameraParams Camera;


public:

    bool calculatePointOnGround(Calibration::Position2D& XYpix, Calibration::RayAndPoint& RayPoint);

    void readCurrentCalibrationParameters(qint32 cameraNumber, const QString& path, qint32 w = 1936, qint32 h = 1216);

    Calibration::ExteriorOr getEO() {return EO;}

    Calibration::SpacecraftPlatform::CAMERA::CameraParams getCameraParams() {return Camera;}

    static CalibrationHelper& instance();


};

#endif // CalibrationHelper_H
