#include "calibrationhelper.h"

CalibrationHelper::CalibrationHelper()
{

}

bool CalibrationHelper::calculatePointOnGround(Calibration::Position2D& XYpix, Calibration::RayAndPoint& RayPoint)
{
    bool success = Calibration::GetRayAndPoint(EO, Camera, XYpix, RayPoint);
    RayPoint.Pos.X = RayPoint.Pos.Z * RayPoint.Vect.X / abs(RayPoint.Vect.Z) + RayPoint.Pos.X;
    RayPoint.Pos.Y = RayPoint.Pos.Z * RayPoint.Vect.Y / abs(RayPoint.Vect.Z) + RayPoint.Pos.Y;
    RayPoint.Pos.X = RayPoint.Pos.Z * RayPoint.Vect.Z / abs(RayPoint.Vect.Z) + RayPoint.Pos.Z;
    return success;
}


void CalibrationHelper::readCurrentCalibrationParameters(qint32 cameraNumber, const QString& path, qint32 w, qint32 h)
{
    QString orPosName = "/pos_orient";
    QFile orPos (path + orPosName);
    QFile otherParams (path + QString("/Cam_%1.x-cam").arg(cameraNumber));
    if (orPos.open(QIODevice::ReadOnly) && otherParams.open(QIODevice::ReadOnly))
    {
        const double pixelSize = 5.85;
        QStringList orientParams;
        QTextStream in(&orPos);
        QString line;
        while (in.readLineInto(&line))
        {
            line.replace(",", ".");
            orientParams = line.split("\t");
            if (orientParams.first().toInt() == cameraNumber)
            {
                orientParams.removeFirst();
                break;
            }
        }

        QStringList params, mainPoint;
        QXmlStreamReader xmlReader(&otherParams);
        QXmlStreamReader::TokenType type;
        while ((type = xmlReader.readNext()) != QXmlStreamReader::TokenType::EndDocument)
        {
            QXmlStreamAttributes list = xmlReader.attributes();
            if (!list.isEmpty())
            {
                if (list.hasAttribute("n"))
                {
                    if (       list.value("n") == "focus"
                               || list.value("n") == "k1"
                               || list.value("n") == "k2"
                               || list.value("n") == "k3"
                               || list.value("n") == "p1"
                               || list.value("n") == "p2"
                               || list.value("n") == "b1"
                               || list.value("n") == "b2")
                    {
                        params.append(list.value("v").toString());
                    }
                    if (list.value("n") == "principal_point")
                    {
                        for (qint32 i = 0; i < 2; ++i)
                        {
                            xmlReader.readNext();
                            mainPoint.append(xmlReader.attributes().value("v").toString());
                            xmlReader.readNext();
                        }
                    }
                }
            }
        }

        Calibration::SpacecraftPlatform::CAMERA::CameraXdirection Dir_R = Calibration::SpacecraftPlatform::CAMERA::CameraXdirection::right;
        Calibration::SpacecraftPlatform::CAMERA::CameraZdirection Dir_Z = Calibration::SpacecraftPlatform::CAMERA::CameraZdirection::frompage;
        Calibration::SpacecraftPlatform::CAMERA::CameraType CamType = Calibration::SpacecraftPlatform::CAMERA::CameraType::central;

        double pixelSizeMk = pixelSize * 1e-3;
        double mainPointX = mainPoint[0].toDouble() / pixelSizeMk  + w / 2;
        double mainPointY = mainPoint[1].toDouble() / pixelSizeMk  + h / 2;
        EO.Init(orientParams[0].toDouble(), orientParams[1].toDouble(), orientParams[2].toDouble(),
                Calibration::Deg2Radian(orientParams[3].toDouble()),
                Calibration::Deg2Radian(orientParams[4].toDouble()),
                Calibration::Deg2Radian(orientParams[5].toDouble()));
        while (params.size() < 8)
        {
            params.append("0");
        }
        Camera.Init(QString::number(cameraNumber), params[0].toDouble(), pixelSize, mainPointX, h - mainPointY, w, h, Dir_R, Dir_Z, CamType,
                params[1].toDouble(), params[2].toDouble(), params[3].toDouble(), params[6].toDouble(), params[7].toDouble(), 0, 0,
                params[4].toDouble(), params[5].toDouble());
    }
}

CalibrationHelper &CalibrationHelper::instance()
{
    static CalibrationHelper obj;
    return obj;
}



