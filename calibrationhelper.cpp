#include "calibrationhelper.h"
#include <math.h>

CalibrationHelper::CalibrationHelper()
{

}

bool CalibrationHelper::calculatePointOnGround(Calibration::Position2D& XYpix, Calibration::RayAndPoint& RayPoint, const QString& name)
{
    QString cam = name.isEmpty() ? currentCamera : name;
    bool success = Calibration::GetRayAndPoint(eos[cam], cameras[cam], XYpix, RayPoint);
    if (success)
    {
        RayPoint.Pos.X = RayPoint.Pos.Z * RayPoint.Vect.X / abs(RayPoint.Vect.Z) + RayPoint.Pos.X;
        RayPoint.Pos.Y = RayPoint.Pos.Z * RayPoint.Vect.Y / abs(RayPoint.Vect.Z) + RayPoint.Pos.Y;
        RayPoint.Pos.Z = RayPoint.Pos.Z * RayPoint.Vect.Z / abs(RayPoint.Vect.Z) + RayPoint.Pos.Z;
    }
    return success;
}

bool CalibrationHelper::getZForHuman(Calibration::Position2D& XYpix, const Calibration::Position& XYZGround, Calibration::Position& XYZHeight, const QString& name)
{
    Calibration::RayAndPoint RayPoint;
    QString cam = name.isEmpty() ? currentCamera : name;
    auto eo = eos[cam];
    bool success = Calibration::GetRayAndPoint(eo, cameras[cam], XYpix, RayPoint);
    if (success)
    {

        double dX = (XYZGround.X - eo.Point.X);
        double dY = (XYZGround.Y - eo.Point.Y);
        double distance_2D = std::sqrt(dX * dX + dY * dY);

        double Norm = sqrt(RayPoint.Vect.X * RayPoint.Vect.X +
                           RayPoint.Vect.Y * RayPoint.Vect.Y);
        //
        XYZHeight.X = RayPoint.Vect.X / Norm * distance_2D + eo.Point.X;
        XYZHeight.Y = RayPoint.Vect.Y / Norm * distance_2D + eo.Point.Y;
        XYZHeight.Z = RayPoint.Vect.Z / Norm * distance_2D + eo.Point.Z;
    }

    return success;
}

void CalibrationHelper::correctHumanHeight(Calibration::Position& XYZHeight, Calibration::Position2D& XYpix, const QString& name)
{
    QString cam = name.isEmpty() ? currentCamera : name;
    Calibration::GetXYfromXYZ(eos[cam], cameras[cam], XYZHeight, XYpix);
}


CalibrationHelper& CalibrationHelper::instance()
{
    static CalibrationHelper obj;
    return obj;
}


void CalibrationHelper::convertToProto(Calibration::ExteriorOr &EO, Calibration::SpacecraftPlatform::CAMERA::CameraParams &Camera, CalibrationParameters &parameters)
{
    parameters.set_b1(Camera.D_b1);
    parameters.set_b2(Camera.D_b2);
    parameters.set_k1(Camera.D_K1);
    parameters.set_k2(Camera.D_K2);
    parameters.set_k3(Camera.D_K3);
    parameters.set_p1(Camera.D_P1);
    parameters.set_p2(Camera.D_P2);
    parameters.set_focus(Camera.focus);
    parameters.set_line(Camera.line);
    parameters.set_lines(Camera.lines);
    parameters.set_sample(Camera.sample);
    parameters.set_samples(Camera.samples);
    parameters.set_pixel_size(Camera.pixelsize);
    parameters.mutable_xyz()->set_x(EO.Point.X);
    parameters.mutable_xyz()->set_y(EO.Point.Y);
    parameters.mutable_xyz()->set_z(EO.Point.Z);
    parameters.mutable_angles()->set_x(EO.OPK.Omega);
    parameters.mutable_angles()->set_y(EO.OPK.Phi);
    parameters.mutable_angles()->set_z(EO.OPK.Kappa);
}

void CalibrationHelper::convertFromProto(const CalibrationParameters &parameters, Calibration::SpacecraftPlatform::CAMERA::CameraParams &Camera, Calibration::ExteriorOr &EO)
{
    Camera.D_b1 = parameters.b1();
    Camera.D_b2 = parameters.b2();
    Camera.D_K1 = parameters.k1();
    Camera.D_K2 = parameters.k2();
    Camera.D_K3 = parameters.k3();
    Camera.D_P1 = parameters.p1();
    Camera.D_P2 = parameters.p2();
    Camera.focus = parameters.focus();
    Camera.line = parameters.line();
    Camera.lines = parameters.lines();
    Camera.sample = parameters.sample();
    Camera.samples = parameters.samples();
    Camera.pixelsize = parameters.pixel_size();
    EO.Point.X = parameters.xyz().x();
    EO.Point.Y = parameters.xyz().y();
    EO.Point.Z = parameters.xyz().z();
    EO.OPK.Omega = parameters.angles().x();
    EO.OPK.Phi = parameters.angles().y();
    EO.OPK.Kappa = parameters.angles().z();
}


void CalibrationHelper::readCurrentCalibrationParameters(const QString& id, const QString& path, Calibration::ExteriorOr& EO,
                                                         Calibration::SpacecraftPlatform::CAMERA::CameraParams& Camera, bool current, qint32 w, qint32 h)
{

    QString fullPath = path + calibrateDir;
    QString orPosName;
    if (!current)
    {
        orPosName = "/pos_orient_standart";
    }
    else
    {
        orPosName = "/pos_orient";
    }
    QFile orPos (fullPath + orPosName);
    QString croppedId = id;
    qint32 pos = id.indexOf("-");
    if (pos != -1)
    {
        croppedId = id.mid(0, id.indexOf("-"));
    }

    QFile otherParams (fullPath + QString("/Cam_%1.x-cam")
                       .arg(croppedId));

    if (orPos.open(QIODevice::ReadOnly) && otherParams.open(QIODevice::ReadOnly))
    {
        double pixelSize;
        QStringList orientParams;

        QTextStream in(&orPos);
        QString line;
        while (in.readLineInto(&line))
        {
            line.replace(",", ".");
            orientParams = line.split("\t");
            if (orientParams.first() == id)
            {
                orientParams.removeFirst();
                break;
            }
        }

        QStringList params, mainPoint, sensorDims;

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
                    else if(list.value("n") == "sensor_dims")
                    {
                        for (qint32 i = 0; i < 2; ++i)
                        {
                            xmlReader.readNext();
                            sensorDims.append(xmlReader.attributes().value("v").toString());
                            xmlReader.readNext();
                        }
                    }
                    else if (list.value("n") == "pixel_size")
                    {
                        xmlReader.readNext();
                        qDebug() << xmlReader.attributes().value("v").toString() << xmlReader.attributes().value("v").toDouble();
                        pixelSize = xmlReader.attributes().value("v").toDouble();
                    }
                }
            }
        }

        Calibration::SpacecraftPlatform::CAMERA::CameraXdirection Dir_R = Calibration::SpacecraftPlatform::CAMERA::CameraXdirection::right;
        Calibration::SpacecraftPlatform::CAMERA::CameraZdirection Dir_Z = Calibration::SpacecraftPlatform::CAMERA::CameraZdirection::frompage;
        Calibration::SpacecraftPlatform::CAMERA::CameraType CamType = Calibration::SpacecraftPlatform::CAMERA::CameraType::central;


        double mainPointX = mainPoint[0].toDouble();
        double mainPointY = mainPoint[1].toDouble();
        if (mainPointX < 1 && mainPointY < 1) // если ещё не сконвертировано в пиксели
        {
            mainPointX = mainPoint[0].toDouble() / pixelSize  + w / 2;
            mainPointY = mainPoint[1].toDouble() / pixelSize  + h / 2;
        }

        EO.Init(orientParams[0].toDouble(), orientParams[1].toDouble(), orientParams[2].toDouble(),
                Calibration::Deg2Radian(orientParams[3].toDouble()),
                Calibration::Deg2Radian(orientParams[4].toDouble()),
                Calibration::Deg2Radian(orientParams[5].toDouble()));
        while (params.size() < 8)
        {
            params.append("0");
        }

        qint32 width = sensorDims.front().toInt();
        qint32 height = sensorDims.back().toInt();

        if (width == 0 && height == 0)
        {
            width = w;
            height = h;
        }

        Camera.Init(id, params[0].toDouble(), pixelSize * 1e3, mainPointX, /*height -*/ mainPointY, width, height, Dir_R, Dir_Z, CamType,
                params[1].toDouble(), params[2].toDouble(), params[3].toDouble(), params[6].toDouble(), params[7].toDouble(), 0, 0,
                params[4].toDouble(), params[5].toDouble());
        Calibration::SpacecraftPlatform::CAMERA::CalcInvDistortion(Camera, 10);
    }
}



