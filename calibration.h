#pragma once

#include <QString>
#include <QVector>
#include <QDebug>


class Calibration
{
public:

    constexpr const static double MaxErr = 0.01;

    enum class Anglesdimentions
    {
        FROMZEROto360, PLUSMINUS180
    };

    enum class ProductType
    {
        OMEGA, KAPPA, PHI
    };
    enum class UnknownTypes
    {
        NoFix, FixALL, FixXYZ
    };
    enum class DistortionCalibrationParams
    {
        NoCalib, Calib_f_dx_dy, Calib_all_Fix_b1_b2, Calib_all_Fix_f, Calib_all,Calib_K123P12
    };

    struct Position
    {
        double X = 0;
        double Y = 0;
        double Z = 0;
    };

    struct Limits
    {
        Limits() : Xmax(0), Ymax(0), Xmin(0), Ymin(0) {}
        int Xmax, Ymax, Xmin, Ymin;
    };

    struct Limits_d
    {
        double Xmax, Xmin, Ymax, Ymin;
        Limits_d() : Xmax(0), Xmin(0), Ymax(0), Ymin(0) {}
        void Init(double mXmax, double mXmin, double mYmax, double mYmin)
        {
            Xmax = mXmax;
            Ymax = mYmax;
            Xmin = mXmin;
            Ymin = mYmin;
        };
    };

    struct Position2D
    {
        Position2D() {}
        Position2D(double _x, double _y) : X(_x), Y(_y) {}
        double X, Y;
    };

    struct PlaneParams
    {
        double A, B, C, D;
    };
    struct RayAndPoint
    {
        Position Pos;
        Position Vect;
    };

    //Повороты из Объектной СК в снимок; порядок повортов такой: 1) Omega, вокруг оси X по часовой, Phi вокруг оси Y по часовой, Kappa вокруг оси Z по часовой
    struct RotAngles
    {
        double Omega = 0;
        double Phi = 0;
        double Kappa = 0;
    };


    struct CameraParams
    {
        RotAngles orient;
        Position pos;
    };

    struct RotationMatrix
    {
        double a1, a2, a3;
        double b1, b2, b3;
        double c1, c2, c3;
        bool isInit;
    };
    struct ExteriorOr
    {
        int ID;
        Position Point;
        RotAngles OPK;
        void Init(double X, double Y, double Z, double Omega, double Phi, double Kappa)
        {

            Point.X = X;
            Point.Y = Y;
            Point.Z = Z;
            OPK.Omega = Omega;
            OPK.Phi = Phi;
            OPK.Kappa = Kappa;
        }


        void Init(const ExteriorOr& EO)
        {
            ID = EO.ID;
            Point.X = EO.Point.X;
            Point.Y = EO.Point.Y;
            Point.Z = EO.Point.Z;
            OPK.Omega = EO.OPK.Omega;
            OPK.Phi = EO.OPK.Phi;
            OPK.Kappa = EO.OPK.Kappa;
        }
    };

    static void LDLT_solver(QVector<double>& R, QVector<double>& B, int N, QVector<double>& Solution);

    static void CalcInverseTransform(RotationMatrix& Rot, RotationMatrix& InvRot);

    static bool VectorIntersect3D(const Position &Pnt1, const Position &Vect1, const Position &Pnt2, const Position &Vect2, Position& Result);

    static bool VectorIntersect3D_v2(const Position& Pnt1, const  Position& Vect1, const  Position& Pnt2, const  Position& Vect2, Position& Result);

    static Position2D ConvertTo2D(const Position& Pnt);

    static PlaneParams PlaneEquationFromPoints(const PlaneParams& Plane1, const Position& Pnt1);

    static PlaneParams PlaneEquationFromPoints(const Position & Pnt1, const Position & Pnt2, const Position & Pnt3);

    static Position XYZtoXYZ(const Position &pnt, const RotationMatrix& matrix);

    static bool Intersection2DVector(const Position2D& Pnt1, const Position2D& Vect1, const Position2D& Pnt2, const Position2D& Vect2,double &t1, double &t2, Position2D& Pnt_res);

    static void ExampleImprooveExtOr();

    //================================================= структуры КА
    class SpacecraftPlatform
    {
    public:


        class CAMERA//================================================= структуры камеры
        {
        public:
            enum class CameraZdirection
            {
                frompage, intopage
            };
            enum class CameraXdirection
            {
                right, left, top, bottom
            };
            enum class CameraType
            {
                central, spherical, cylindrical
            };
            enum class CameraDistortionType
            {
                Classical, IKIStyle
            };
            //======================== для панорамной камеры размер пикселя задается в градусах на / пиксел
            struct CameraParams
            {
                QString name;
                double focus;
                double pixelsize;
                double sample;
                double line;
                double samples;
                double lines;
                CameraXdirection x_direction;
                CameraZdirection z_direction;
                CameraType cameraType;
                CameraDistortionType m_DistortionType;
                Limits m_Lims_return;
                double D_K1 = 0, D_K2 = 0, D_K3 = 0, D_b1 = 0, D_b2 = 0, D_a1 = 0, D_a2 = 0, D_P1 = 0, D_P2 = 0;
                double D_Inv_K1 = 0, D_Inv_K2 = 0, D_Inv_K3 = 0, D_Inv_b1 = 0, D_Inv_b2 = 0;
                double D_Inv_a1 = 0, D_Inv_a2 = 0, D_Inv_P1 = 0, D_Inv_P2 = 0,D_Inv_dx = 0, D_Inv_dy = 0;
                double D_ax0 = 0, D_ax1 = 0, D_ax2 = 0, D_ax3 = 0, D_ax4 = 0, D_ax5 = 0, D_ax6 = 0, D_ax7 = 0, D_ax8 = 0, D_ax9 = 0;
                double D_ay0 = 0, D_ay1 = 0, D_ay2 = 0, D_ay3 = 0, D_ay4 = 0, D_ay5 = 0, D_ay6 = 0, D_ay7 = 0, D_ay8 = 0, D_ay9 = 0;
                double D_Inv_ax0 = 0, D_Inv_ax1 = 0, D_Inv_ax2 = 0, D_Inv_ax3 = 0, D_Inv_ax4 = 0;
                double D_Inv_ax5 = 0, D_Inv_ax6 = 0, D_Inv_ax7 = 0, D_Inv_ax8 = 0, D_Inv_ax9= 0;
                double D_Inv_ay0 = 0, D_Inv_ay1 = 0, D_Inv_ay2 = 0, D_Inv_ay3 = 0, D_Inv_ay4 = 0;
                double D_Inv_ay5 = 0, D_Inv_ay6 = 0, D_Inv_ay7 = 0, D_Inv_ay8 = 0, D_Inv_ay9= 0;



                void Init(QString m_name, double m_focus, double m_pixelsize, double m_sample, double m_line, double m_samples, double m_lines,
                          CAMERA::CameraXdirection m_x_direction, CAMERA::CameraZdirection m_z_direction, CAMERA::CameraType m_cameraType, CAMERA::CameraDistortionType m_DistT)
                {
                    name = m_name;
                    focus = m_focus;
                    pixelsize = m_pixelsize;
                    sample = m_sample;
                    line = m_line;
                    samples = m_samples;
                    lines = m_lines;
                    x_direction = m_x_direction;
                    z_direction = m_z_direction;
                    cameraType = m_cameraType;
                    m_DistortionType = m_DistT;
                }
                void Init(QString m_name, double m_focus, double m_pixelsize, double m_sample, double m_line, double m_samples, double m_lines,
                          CAMERA::CameraXdirection m_x_direction, CAMERA::CameraZdirection m_z_direction, CAMERA::CameraType m_cameraType,
                          double m_D_K1, double m_D_K2, double m_D_K3, double m_D_b1, double m_D_b2, double m_D_a1, double m_D_a2, double m_D_P1, double m_D_P2)
                {
                    name = m_name;
                    focus = m_focus;
                    pixelsize = m_pixelsize;
                    sample = m_sample;
                    line = m_line;
                    samples = m_samples;
                    lines = m_lines;
                    x_direction = m_x_direction;
                    z_direction = m_z_direction;
                    cameraType = m_cameraType;
                    m_DistortionType = CameraDistortionType::Classical;
                    D_K1 = m_D_K1;
                    D_K2 = m_D_K2;
                    D_K3 = m_D_K3;
                    D_b1 = m_D_b1;
                    D_b2 = m_D_b2;
                    D_a1 = m_D_a1;
                    D_a2 = m_D_a2;
                    D_P1 = m_D_P1;
                    D_P2 = m_D_P2;
                }
                void Init(QString m_name, double m_focus, double m_pixelsize, double m_sample, double m_line, double m_samples, double m_lines,
                          CAMERA::CameraXdirection m_x_direction, CAMERA::CameraZdirection m_z_direction, CAMERA::CameraType m_cameraType,
                          double m_ax0, double m_ax1, double m_ax2, double m_ax3, double m_ax4, double m_ax5, double m_ax6, double m_ax7, double m_ax8, double m_ax9,
                          double m_ay0, double m_ay1, double m_ay2, double m_ay3, double m_ay4, double m_ay5, double m_ay6, double m_ay7, double m_ay8, double m_ay9,
                          double m_Inv_ax0, double m_Inv_ax1, double m_Inv_ax2, double m_Inv_ax3, double m_Inv_ax4, double m_Inv_ax5, double m_Inv_ax6, double m_Inv_ax7, double m_Inv_ax8, double m_Inv_ax9,
                          double m_Inv_ay0, double m_Inv_ay1, double m_Inv_ay2, double m_Inv_ay3, double m_Inv_ay4, double m_Inv_ay5, double m_Inv_ay6, double m_Inv_ay7, double m_Inv_ay8, double m_Inv_ay9)
                {
                    name = m_name;
                    focus = m_focus;
                    pixelsize = m_pixelsize;
                    sample = m_sample;
                    line = m_line;
                    samples = m_samples;
                    lines = m_lines;
                    x_direction = m_x_direction;
                    z_direction = m_z_direction;
                    cameraType = m_cameraType;
                    m_DistortionType = CameraDistortionType::IKIStyle;
                    D_ax0 = m_ax0; D_ay0 = m_ay0; D_Inv_ax0 = m_Inv_ax0; D_Inv_ay0 = m_Inv_ay0;
                    D_ax1 = m_ax1; D_ay1 = m_ay1; D_Inv_ax1 = m_Inv_ax1; D_Inv_ay1 = m_Inv_ay1;
                    D_ax2 = m_ax2; D_ay2 = m_ay2; D_Inv_ax2 = m_Inv_ax2; D_Inv_ay2 = m_Inv_ay2;
                    D_ax3 = m_ax3; D_ay3 = m_ay3; D_Inv_ax3 = m_Inv_ax3; D_Inv_ay3 = m_Inv_ay3;
                    D_ax4 = m_ax4; D_ay4 = m_ay4; D_Inv_ax4 = m_Inv_ax4; D_Inv_ay4 = m_Inv_ay4;
                    D_ax5 = m_ax5; D_ay5 = m_ay5; D_Inv_ax5 = m_Inv_ax5; D_Inv_ay5 = m_Inv_ay5;
                    D_ax6 = m_ax6; D_ay6 = m_ay6; D_Inv_ax6 = m_Inv_ax6; D_Inv_ay6 = m_Inv_ay6;
                    D_ax7 = m_ax7; D_ay7 = m_ay7; D_Inv_ax7 = m_Inv_ax7; D_Inv_ay7 = m_Inv_ay7;
                    D_ax8 = m_ax8; D_ay8 = m_ay8; D_Inv_ax8 = m_Inv_ax8; D_Inv_ay8 = m_Inv_ay8;
                    D_ax9 = m_ax9; D_ay9 = m_ay9; D_Inv_ax9 = m_Inv_ax9; D_Inv_ay9 = m_Inv_ay9;
                }


                void Init(const CAMERA::CameraParams& Camera)
                {
                    name = Camera.name;
                    focus = Camera.focus;
                    pixelsize = Camera.pixelsize;
                    sample = Camera.sample;
                    line = Camera.line;
                    samples = Camera.samples;
                    lines = Camera.lines;
                    x_direction = Camera.x_direction;
                    z_direction = Camera.z_direction;
                    cameraType = Camera.cameraType;
                    m_DistortionType = Camera.m_DistortionType;
                    D_K1 = Camera.D_K1;
                    D_K2 = Camera.D_K2;
                    D_K3 = Camera.D_K3;
                    D_b1 = Camera.D_b1;
                    D_b2 = Camera.D_b2;
                    D_a1 = Camera.D_a1;
                    D_a2 = Camera.D_a2;
                    D_P1 = Camera.D_P1;
                    D_P2 = Camera.D_P2;
                    D_ax0 = Camera.D_ax0; D_ay0 = Camera.D_ay0; D_Inv_ax0 = Camera.D_Inv_ax0; D_Inv_ay0 = Camera.D_Inv_ay0;
                    D_ax1 = Camera.D_ax1; D_ay1 = Camera.D_ay1; D_Inv_ax1 = Camera.D_Inv_ax1; D_Inv_ay1 = Camera.D_Inv_ay1;
                    D_ax2 = Camera.D_ax2; D_ay2 = Camera.D_ay2; D_Inv_ax2 = Camera.D_Inv_ax2; D_Inv_ay2 = Camera.D_Inv_ay2;
                    D_ax3 = Camera.D_ax3; D_ay3 = Camera.D_ay3; D_Inv_ax3 = Camera.D_Inv_ax3; D_Inv_ay3 = Camera.D_Inv_ay3;
                    D_ax4 = Camera.D_ax4; D_ay4 = Camera.D_ay4; D_Inv_ax4 = Camera.D_Inv_ax4; D_Inv_ay4 = Camera.D_Inv_ay4;
                    D_ax5 = Camera.D_ax5; D_ay5 = Camera.D_ay5; D_Inv_ax5 = Camera.D_Inv_ax5; D_Inv_ay5 = Camera.D_Inv_ay5;
                    D_ax6 = Camera.D_ax6; D_ay6 = Camera.D_ay6; D_Inv_ax6 = Camera.D_Inv_ax6; D_Inv_ay6 = Camera.D_Inv_ay6;
                    D_ax7 = Camera.D_ax7; D_ay7 = Camera.D_ay7; D_Inv_ax7 = Camera.D_Inv_ax7; D_Inv_ay7 = Camera.D_Inv_ay7;
                    D_ax8 = Camera.D_ax8; D_ay8 = Camera.D_ay8; D_Inv_ax8 = Camera.D_Inv_ax8; D_Inv_ay8 = Camera.D_Inv_ay8;
                    D_ax9 = Camera.D_ax9; D_ay9 = Camera.D_ay9; D_Inv_ax9 = Camera.D_Inv_ax9; D_Inv_ay9 = Camera.D_Inv_ay9;
                }


            };
            static bool FromImage2Cam(double sample, double line, CAMERA::CameraParams& Camera, Position& vect);
            static bool FromCam2Image(Position& vect, const CameraParams &Camera, double& sample, double& line);
            static bool CalcInvDistortion(CAMERA::CameraParams& Camera, int step);
        };
    public:

        static void FromCam2SpacecraftPlatform(Position& inCAM, RotationMatrix& newMatrix, Position& inSpacecraftPlatform);

        static void FromSpacecraftPlatform2Cam(Position& inSpacecraftPlatform, RotationMatrix& newMatrix, Position& inCAM);
    };

    static bool GetXYZfromXYXY(ExteriorOr& EO_left, ExteriorOr& EO_right,
                               SpacecraftPlatform::CAMERA::CameraParams& Camera_left, SpacecraftPlatform::CAMERA::CameraParams& Camera_right,
                               Position2D& XYpix_left, Position2D& XYpix_right, Position&  XYZ);
    static bool GetXYfromXYZ(const ExteriorOr &EO, const SpacecraftPlatform::CAMERA::CameraParams &Camera, Position& XYZ, Position2D& XYpix);
    static bool GetRayAndPoint(ExteriorOr& EO, SpacecraftPlatform::CAMERA::CameraParams& Camera, Position2D& XYpix, RayAndPoint& RayPoint);

    static double Radian2Deg(double Val);
    static double Deg2Radian(double Val);
    static double AngleBetweenVectors(const Position &V1, const Position &V2);

    static double QuaterOfAngle(double A, double B, Anglesdimentions VAL);
    static void Matrix2OmegaPhiKappa(RotationMatrix& Matrix, RotAngles& OPK);
    static void OmegaPhiKappa2Matrix(const RotAngles &OPK, RotationMatrix&  Matrix);

    static void OmegaPhiKappa2Product(RotAngles& OPK, ProductType ptype, RotationMatrix& Matrix);
    static QVector <double> MatrixTranspone(const QVector<double> &InitMat, int Rows, int Cols);
    static QVector <double> MatrixMultyply(QVector <double>& InitMat1, int Rows1, int Cols1, const QVector<double> &InitMat2, int Rows2, int Cols2);
    static double GetAbsDist3d(QVector <double>& vect);



    class Adjustment
    {

    private:
        const double Precision = 1E-13;

    public:

        enum pType {Tie, GCPs};
        struct image
        {
            int ImageID;
            SpacecraftPlatform::CAMERA::CameraParams Camera;
            ExteriorOr Eo;
        };
        struct measure
        {
            int camID;
            int GCPsID;
            image* imageID;
            pType type;
            Calibration::Position2D meas;
            Calibration::Position XYZg;
        };
        struct Data
        {
            DistortionCalibrationParams CalibParams;
            UnknownTypes UType;
            int image_counts;
            int meas_counts;
            int rot_and_shift;
            QVector <image> images;
            QVector <measure> measurements;
        };
        struct measProduct
        {
            double dx_dXs, dx_dYs, dx_dZs, dy_dXs, dy_dYs, dy_dZs, dx_dOm, dy_dOm, dx_dPh, dy_dPh, dx_dKa, dy_dKa;
            double dx_dF, dy_dF;
            //Classical
            double dx_dx0, dx_dy0, dx_dK1, dx_dK2, dx_dK3, dx_dP1, dx_dP2, dx_db1, dx_db2;
            double dy_dx0, dy_dy0, dy_dK1, dy_dK2, dy_dK3, dy_dP1, dy_dP2;// , dy_db1, dy_db2;
            //IKI STYLE
            double dx_ax0, dx_ax1, dx_ax2, dx_ax3, dx_ax4, dx_ax5, dx_ax6, dx_ax7, dx_ax8, dx_ax9;
            double dy_ax0, dy_ax1, dy_ax2, dy_ax3, dy_ax4, dy_ax5, dy_ax6, dy_ax7, dy_ax8, dy_ax9;
        };

        int GetUnknownDistortion(Data& information, QVector <bool>& Rows)
        {
            int unkonws = 0;
            if (information.CalibParams != DistortionCalibrationParams::NoCalib)
            {
                if (!Rows.isEmpty())
                {
                    for (int i = 0; i < Rows.size(); i++)
                        Rows[i] = false;
                }
                if (information.measurements[0].imageID->Camera.m_DistortionType == SpacecraftPlatform::CAMERA::CameraDistortionType::Classical)
                {
                    if (information.CalibParams == DistortionCalibrationParams::Calib_all)
                    {
                        unkonws = 10;
                        if (!Rows.isEmpty())
                        {
                            for (int i = 0; i < unkonws; i++)
                                Rows[i] = true;
                        }
                    }
                    else if (information.CalibParams == DistortionCalibrationParams::Calib_all_Fix_b1_b2)
                    {
                        unkonws = 10 - 2;
                        if (!Rows.isEmpty())
                        {
                            for (int i = 0; i < unkonws; i++)
                                Rows[i] = true;
                        }
                    }
                    else if (information.CalibParams == DistortionCalibrationParams::Calib_all_Fix_f)
                    {
                        unkonws = 10 - 1;
                        if (!Rows.isEmpty())
                        {
                            for (int i = 1; i < unkonws + 1; i++)
                                Rows[i] = true;
                        }
                    }
                    else if (information.CalibParams == DistortionCalibrationParams::Calib_f_dx_dy)
                    {
                        unkonws = 3;
                        if (!Rows.isEmpty())
                        {
                            for (int i = 0; i < unkonws; i++)
                                Rows[i] = true;
                        }
                    }
                    else if (information.CalibParams == DistortionCalibrationParams::Calib_K123P12)
                    {
                        unkonws = 5;
                        if (!Rows.isEmpty())
                        {
                            for (int i = 3; i < 3+unkonws; i++)
                                Rows[i] = true;
                        }
                    }
                }
                else
                {
                    if (information.CalibParams == DistortionCalibrationParams::Calib_all)
                        unkonws = 1 + 10 * 2;
                    else
                    {
                        unkonws = 0;
                        information.CalibParams = DistortionCalibrationParams::NoCalib;
                    }
                }
            }
            return(unkonws);
        };

        int GetTotalNoGCPs(Data& information)
        {
            int unkonws = 0;
            for (int i = 0; i < information.meas_counts; i++)
            {
                if (information.measurements[i].type == pType::Tie) { unkonws += 3; }
            }
            return(unkonws);
        }
        int GetUnknownParams(Data& information)
        {
            int rot_and_shift = 6;
            if(information.UType == UnknownTypes::NoFix)
                rot_and_shift = 6;
            else if (information.UType == UnknownTypes::FixALL)
                rot_and_shift = 0;
            else if (information.UType == UnknownTypes::FixXYZ)
                rot_and_shift = 3;

            information.rot_and_shift = rot_and_shift;
            QVector <bool> dummy;
            int unkonws = information.image_counts * rot_and_shift + GetTotalNoGCPs(information) + GetUnknownDistortion(information, dummy);
            return(unkonws);
        };
        //=======================================================================
        void CalcProducts(measure& meas, measProduct& prod, double &Lx, double &Ly)
        {
            double focus = meas.imageID->Camera.focus / 1000;//mm
            Calibration::RotationMatrix rmat;
            Calibration::RotationMatrix pmat;
            //===============================
            double Xm = meas.XYZg.X;
            double Ym = meas.XYZg.Y;
            double Zm = meas.XYZg.Z;
            //===============================
            double Xs = meas.imageID->Eo.Point.X;
            double Ys = meas.imageID->Eo.Point.Y;
            double Zs = meas.imageID->Eo.Point.Z;
            //===============================
            Calibration::OmegaPhiKappa2Matrix(meas.imageID->Eo.OPK, rmat);
            //===============================
            double X_ = (rmat.a1 * (Xm - Xs) + rmat.b1*(Ym - Ys) + rmat.c1*(Zm - Zs));
            double Y_ = (rmat.a2 * (Xm - Xs) + rmat.b2*(Ym - Ys) + rmat.c2*(Zm - Zs));
            double Z_ = (rmat.a3 * (Xm - Xs) + rmat.b3*(Ym - Ys) + rmat.c3*(Zm - Zs));
            //===============================
            double f_Z2 = -(focus) / (Z_*Z_);
            //===============================
            Lx = (-focus * X_ / Z_);
            Ly = (-focus * Y_ / Z_);
            //===============================
            prod.dx_dXs = f_Z2 * (-rmat.a1*Z_ + rmat.a3*X_);//dx_dXs
            prod.dx_dYs = f_Z2 * (-rmat.b1*Z_ + rmat.b3*X_);//dx_dYs
            prod.dx_dZs = f_Z2 * (-rmat.c1*Z_ + rmat.c3*X_);//dx_dZs
            //===============================
            prod.dy_dXs = f_Z2 * (-rmat.a2*Z_ + rmat.a3*Y_);//dy_dXs
            prod.dy_dYs = f_Z2 * (-rmat.b2*Z_ + rmat.b3*Y_);//dy_dYs
            prod.dy_dZs = f_Z2 * (-rmat.c2*Z_ + rmat.c3*Y_);//dy_dZs
            //===============================
            OmegaPhiKappa2Product(meas.imageID->Eo.OPK, Calibration::ProductType::OMEGA, pmat);//= focus * X_* Y_ / (Z_*Z_);
            prod.dx_dOm = f_Z2 * ((pmat.a1*(Xm - Xs) + pmat.b1*(Ym - Ys) + pmat.c1*(Zm - Zs))* Z_ - (pmat.a3*(Xm - Xs) + pmat.b3*(Ym - Ys) + pmat.c3*(Zm - Zs))*X_);//dx_dOm
            prod.dy_dOm = f_Z2 * ((pmat.a2*(Xm - Xs) + pmat.b2*(Ym - Ys) + pmat.c2*(Zm - Zs))* Z_ - (pmat.a3*(Xm - Xs) + pmat.b3*(Ym - Ys) + pmat.c3*(Zm - Zs))*Y_);//dy_dOm
            //===============================
            OmegaPhiKappa2Product(meas.imageID->Eo.OPK, ProductType::PHI, pmat);
            prod.dx_dPh = f_Z2 * ((pmat.a1*(Xm - Xs) + pmat.b1*(Ym - Ys) + pmat.c1*(Zm - Zs))* Z_ - (pmat.a3*(Xm - Xs) + pmat.b3*(Ym - Ys) + pmat.c3*(Zm - Zs))*X_);//dx_dPh
            prod.dy_dPh = f_Z2 * ((pmat.a2*(Xm - Xs) + pmat.b2*(Ym - Ys) + pmat.c2*(Zm - Zs))* Z_ - (pmat.a3*(Xm - Xs) + pmat.b3*(Ym - Ys) + pmat.c3*(Zm - Zs))*Y_);//dy_dPh
            //===============================
            OmegaPhiKappa2Product(meas.imageID->Eo.OPK, ProductType::KAPPA, pmat);
            prod.dx_dKa = f_Z2 * ((pmat.a1*(Xm - Xs) + pmat.b1*(Ym - Ys) + pmat.c1*(Zm - Zs))* Z_ - (pmat.a3*(Xm - Xs) + pmat.b3*(Ym - Ys) + pmat.c3*(Zm - Zs))*X_);//dx_dKa
            prod.dy_dKa = f_Z2 * ((pmat.a2*(Xm - Xs) + pmat.b2*(Ym - Ys) + pmat.c2*(Zm - Zs))* Z_ - (pmat.a3*(Xm - Xs) + pmat.b3*(Ym - Ys) + pmat.c3*(Zm - Zs))*Y_);//dy_dKa
            //=============================== Calbration
            /*
    SpacecraftPlatform::CAMERA::CameraParams^ CamNoDistor = gcnew SpacecraftPlatform::CAMERA::CameraParams();
    CamNoDistor.cameraType = meas.imageID->Camera.cameraType;
    CamNoDistor.focus = meas.imageID->Camera.focus;
    CamNoDistor.pixelsize = meas.imageID->Camera.pixelsize;
    CamNoDistor.sample = meas.imageID->Camera.sample;
    CamNoDistor.samples = meas.imageID->Camera.samples;
    CamNoDistor.line = meas.imageID->Camera.line;
    CamNoDistor.lines = meas.imageID->Camera.lines;
    CamNoDistor.m_DistortionType = meas.imageID->Camera.m_DistortionType;
    CamNoDistor.x_direction = meas.imageID->Camera.x_direction;
    CamNoDistor.z_direction = meas.imageID->Camera.z_direction;
    //======
    */
            Position noditor;
            SpacecraftPlatform::CAMERA::FromImage2Cam(meas.meas.X, meas.meas.Y, meas.imageID->Camera, noditor);
            noditor.X *= 1000;
            noditor.Y *= 1000;
            noditor.Z *= 1000;
            //============================================== f
            prod.dx_dF = -X_ / Z_; prod.dy_dF = -Y_ / Z_;
            //==============================================
            if (meas.imageID->Camera.m_DistortionType == SpacecraftPlatform::CAMERA::CameraDistortionType::Classical)
            {
                //==============================================
                double R2 = noditor.X*noditor.X + noditor.Y*noditor.Y;//m
                double R4 = R2 * R2;
                double R6 = R4 * R2;
                //==============================================  X0 , Y0
                prod.dx_dx0 = 1.0; prod.dy_dx0 = 0.0;
                prod.dx_dy0 = 0.0; prod.dy_dy0 = 1.0;
                //=====//(1 - D_K1 * R^2 - D_K2 * R^4 - D_K3 * R^6) * (x-x0) = -f*X/Z =======> x = x0 + (x-x0)(D_K1 * R^2 + D_K2 * R^4 + D_K3 * R^6) - f*X/Z
                prod.dx_dK1 = noditor.X*R2; prod.dy_dK1 = noditor.Y*R2;
                prod.dx_dK2 = noditor.X*R4; prod.dy_dK2 = noditor.Y*R4;
                prod.dx_dK3 = noditor.X*R6; prod.dy_dK3 = noditor.Y*R6;
                //======== P1, P2
                prod.dx_dP1 = (2.*noditor.X*noditor.X + R2); prod.dy_dP1 = (2.*noditor.X*noditor.Y);
                prod.dx_dP2 = (2.*noditor.X*noditor.Y);      prod.dy_dP2 = (2.*noditor.Y*noditor.Y + R2);
                //======== b1,b2
                prod.dx_db1 = noditor.X;
                prod.dx_db2 = noditor.Y;
            }
            else if (meas.imageID->Camera.m_DistortionType == SpacecraftPlatform::CAMERA::CameraDistortionType::IKIStyle)
            {
                //=====//x + ax0 + ax1*x + ax2*y + ax3*x2 + ax4*xy + ax5*y2 + ax6*x3 + ax7*x2y + ax8*xy2 + ax9*y3 = -f * X/Z
                prod.dx_ax0 = -1.0;
                prod.dx_ax1 = -noditor.X;
                prod.dx_ax2 = -noditor.Y;
                prod.dx_ax3 = -noditor.X*noditor.X;
                prod.dx_ax4 = -noditor.X*noditor.Y;
                prod.dx_ax5 = -noditor.Y*noditor.Y;
                prod.dx_ax6 = -noditor.X*noditor.X*noditor.X;
                prod.dx_ax7 = -noditor.X*noditor.X*noditor.Y;
                prod.dx_ax8 = -noditor.X*noditor.Y*noditor.Y;
                prod.dx_ax9 = -noditor.Y*noditor.Y*noditor.Y;
                //
                prod.dy_ax0 = -1.0;
                prod.dy_ax1 = -noditor.X;
                prod.dy_ax2 = -noditor.Y;
                prod.dy_ax3 = -noditor.X*noditor.X;
                prod.dy_ax4 = -noditor.X*noditor.Y;
                prod.dy_ax5 = -noditor.Y*noditor.Y;
                prod.dy_ax6 = -noditor.X*noditor.X*noditor.X;
                prod.dy_ax7 = -noditor.X*noditor.X*noditor.Y;
                prod.dy_ax8 = -noditor.X*noditor.Y*noditor.Y;
                prod.dy_ax9 = -noditor.Y*noditor.Y*noditor.Y;
            }
        }
        //=======================================================================
        //        void Adjust(Data& information)
        //        {
        //            bool use_5_params = true;
        //            int gcps = GetTotalNoGCPs(information);
        //            int unknowns = GetUnknownParams(information, false);
        //            if (unknowns == 0 || information.meas_counts*2 <= unknowns)
        //                return;
        //            //========================================
        //            QVector <double> Solution;
        //            //========================================
        //            QVector <double> A (unknowns*information.meas_counts * 2);
        //            QVector <double> L (information.meas_counts * 2);
        //            //========================================
        //            measProduct prod;
        //            Position  vXYZ_m;
        //            //==========================================================
        //            /*
        //    for (int i = 0; i < information.meas_counts; i++)
        //    {
        //      SpacecraftPlatform::CAMERA::FromImage2Cam(information.measurements[i].meas.X, information.measurements[i].meas.Y, information.measurements[i].imageID->Camera, vXYZ_m);
        //      vXYZ.X += vXYZ_m.X;
        //      vXYZ.Y += vXYZ_m.Y;
        //      vXYZ.Z += vXYZ_m.Z;
        //    }
        //    vXYZ.X /= information.meas_counts;
        //    vXYZ.Y /= information.meas_counts;
        //    vXYZ.Z /= information.meas_counts;
        //    Position^ V2 = gcnew Position();
        //    V2.X = 0; V2.Y = 0; V2.Z = -1;
        //    double omega = Deg2Radian(AngleBetweenVectors(vXYZ, V2));
        //    if (information.images[0].Eo.OPK.Omega == 0)
        //      information.images[0].Eo.OPK.Omega = omega;
        //    */
        //            //==========================================================
        //            int image_params = 6;
        //            for (int iter=0;iter < 1000;iter++)
        //            {
        //                //=================================
        //                int row = 0;
        //                double Lx, Ly;
        //                for (int i = 0; i < information.meas_counts; i++)
        //                {
        //                    CalcProducts(information.measurements[i], prod, Lx, Ly);
        //                    //===========================================================================================
        //                    //GetXYfromXYZ(information.measurements[i].imageID->Eo, information.measurements[i].imageID->Camera, information.measurements[i].XYZg, XY);
        //                    //SpacecraftPlatform::CAMERA::FromImage2Cam(XY.X, XY.Y, information.measurements[i].imageID->Camera, vXYZ);
        //                    SpacecraftPlatform::CAMERA::FromImage2Cam(information.measurements[i].meas.X, information.measurements[i].meas.Y, information.measurements[i].imageID->Camera, vXYZ_m);
        //                    //===========================================================================================
        //                    /*L[row] = vXYZ.X - vXYZ_m.X;
        //        L[row + 1] = vXYZ.Y - vXYZ_m.Y;
        //        */
        //                    L[row]     = (Lx - vXYZ_m.X);
        //                    L[row + 1] = (Ly - vXYZ_m.Y);
        //                    //===========================================================================================
        //                    if (information.measurements[i].type == pType::Tie)
        //                    {
        //                        A[row*unknowns + information.image_counts * image_params + information.measurements[i].GCPsID * 3 + 0] = -prod.dx_dXs;
        //                        A[row*unknowns + information.image_counts * image_params + information.measurements[i].GCPsID * 3 + 1] = -prod.dx_dYs;
        //                        A[row*unknowns + information.image_counts * image_params + information.measurements[i].GCPsID * 3 + 2] = -prod.dx_dZs;
        //                        A[(row + 1)*unknowns + information.image_counts * image_params + information.measurements[i].GCPsID * 3 + 0] = -prod.dy_dXs;
        //                        A[(row + 1)*unknowns + information.image_counts * image_params + information.measurements[i].GCPsID * 3 + 1] = -prod.dy_dYs;
        //                        A[(row + 1)*unknowns + information.image_counts * image_params + information.measurements[i].GCPsID * 3 + 2] = -prod.dy_dZs;
        //                    }
        //                    //===========================================================================================
        //                    {
        //                        int tx = 0;
        //                        int ty = 0;
        //                        //====== shift
        //                        A[row*unknowns + information.measurements[i].imageID->ImageID * image_params + tx] = prod.dx_dXs; tx++;
        //                        A[row*unknowns + information.measurements[i].imageID->ImageID * image_params + tx] = prod.dx_dYs; tx++;
        //                        A[row*unknowns + information.measurements[i].imageID->ImageID * image_params + tx] = prod.dx_dZs; tx++;
        //                        A[(row + 1)*unknowns + information.measurements[i].imageID->ImageID * image_params + ty] = prod.dy_dXs; ty++;
        //                        A[(row + 1)*unknowns + information.measurements[i].imageID->ImageID * image_params + ty] = prod.dy_dYs; ty++;
        //                        A[(row + 1)*unknowns + information.measurements[i].imageID->ImageID * image_params + ty] = prod.dy_dZs; ty++;
        //                        //====== rotation
        //                        A[row*unknowns + information.measurements[i].imageID->ImageID * image_params + tx] = prod.dx_dOm; tx++;
        //                        A[row*unknowns + information.measurements[i].imageID->ImageID * image_params + tx] = prod.dx_dPh; tx++;
        //                        A[row*unknowns + information.measurements[i].imageID->ImageID * image_params + tx] = prod.dx_dKa; tx++;
        //                        A[(row + 1)*unknowns + information.measurements[i].imageID->ImageID * image_params + ty] = prod.dy_dOm; ty++;
        //                        A[(row + 1)*unknowns + information.measurements[i].imageID->ImageID * image_params + ty] = prod.dy_dPh; ty++;
        //                        A[(row + 1)*unknowns + information.measurements[i].imageID->ImageID * image_params + ty] = prod.dy_dKa; ty++;
        //                        //======= calibration
        //                        if (information.detectdistortion)
        //                        {
        //                            tx = 0;
        //                            ty = 0;
        //                            if (information.measurements[i].imageID->Camera.m_DistortionType == SpacecraftPlatform::CAMERA::CameraDistortionType::Classical)
        //                            {
        //                                //x
        //                                A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_dF; tx++;
        //                                A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_dx0; tx++;
        //                                //A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_dy0; tx++;
        //                                A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_dK1; tx++;
        //                                A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_dK2; tx++;
        //                                A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_dK3; tx++;
        //                                A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_dP1; tx++;
        //                                A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_dP2; tx++;
        //                                //A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_db1; tx++;
        //                                //A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_db2; tx++;
        //                                //y
        //                                A[(row + 1)*unknowns + information.image_counts * image_params + gcps + ty] = prod.dy_dF; ty++;
        //                                A[(row + 1)*unknowns + information.image_counts * image_params + gcps + ty] = prod.dy_dx0; ty++;
        //                                //A[(row + 1)*unknowns + information.image_counts * image_params + gcps + ty] = prod.dy_dy0; ty++;
        //                                A[(row + 1)*unknowns + information.image_counts * image_params + gcps + ty] = prod.dy_dK1; ty++;
        //                                A[(row + 1)*unknowns + information.image_counts * image_params + gcps + ty] = prod.dy_dK2; ty++;
        //                                A[(row + 1)*unknowns + information.image_counts * image_params + gcps + ty] = prod.dy_dK3; ty++;
        //                                A[(row + 1)*unknowns + information.image_counts * image_params + gcps + ty] = prod.dy_dP1; ty++;
        //                                A[(row + 1)*unknowns + information.image_counts * image_params + gcps + ty] = prod.dy_dP2; ty++;
        //                            }
        //                            else if (information.measurements[i].imageID->Camera.m_DistortionType == SpacecraftPlatform::CAMERA::CameraDistortionType::IKIStyle)
        //                            {
        //                                //x
        //                                A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_dF; tx++;
        //                                A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_ax0; tx++;
        //                                //A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_ax1; tx++;
        //                                //A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_ax2; tx++;
        //                                A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_ax3; tx++;
        //                                A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_ax4; tx++;
        //                                A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_ax5; tx++;
        //                                A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_ax6; tx++;
        //                                if (!use_5_params)
        //                                {
        //                                    A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_ax7; tx++;
        //                                    A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_ax8; tx++;
        //                                    A[row*unknowns + information.image_counts * image_params + gcps + tx] = prod.dx_ax9; tx++;
        //                                }
        //                                //y
        //                                A[(row + 1)*unknowns + information.image_counts * image_params + gcps + 0] = prod.dy_dF;
        //                                A[(row + 1)*unknowns + information.image_counts * image_params + gcps + tx] = prod.dy_ax0; tx++;
        //                                //A[(row + 1)*unknowns + information.image_counts * image_params + gcps + tx] = prod.dy_ax1; tx++;
        //                                //A[(row + 1)*unknowns + information.image_counts * image_params + gcps + tx] = prod.dy_ax2; tx++;
        //                                A[(row + 1)*unknowns + information.image_counts * image_params + gcps + tx] = prod.dy_ax3; tx++;
        //                                A[(row + 1)*unknowns + information.image_counts * image_params + gcps + tx] = prod.dy_ax4; tx++;
        //                                A[(row + 1)*unknowns + information.image_counts * image_params + gcps + tx] = prod.dy_ax5; tx++;
        //                                A[(row + 1)*unknowns + information.image_counts * image_params + gcps + tx] = prod.dy_ax6; tx++;
        //                                if (!use_5_params)
        //                                {
        //                                    A[(row + 1)*unknowns + information.image_counts * image_params + gcps + tx] = prod.dy_ax7; tx++;
        //                                    A[(row + 1)*unknowns + information.image_counts * image_params + gcps + tx] = prod.dy_ax8; tx++;
        //                                    A[(row + 1)*unknowns + information.image_counts * image_params + gcps + tx] = prod.dy_ax9; tx++;
        //                                }
        //                            }
        //                        }
        //                    }
        //                    //===========================================================================================
        //                    row += 2;
        //                }
        //                //========================================
        //                QVector <double> AT = MatrixTranspone(A, information.meas_counts * 2, unknowns);
        //                QVector  <double> R = MatrixMultyply(AT, unknowns, information.meas_counts * 2, A, information.meas_counts * 2, unknowns);
        //                QVector <double> B = MatrixMultyply(AT, unknowns, information.meas_counts * 2, L, information.meas_counts * 2, 1);
        //                LDLT_solver(R, B, unknowns, Solution);
        //                //========================================
        //                for (int i = 0; i < information.image_counts; i++)
        //                {
        //                    int tx = 0;
        //                    information.images[i].Eo.Point.X -= Solution[information.images[i].ImageID * image_params + tx]; tx++;
        //                    information.images[i].Eo.Point.Y -= Solution[information.images[i].ImageID * image_params + tx]; tx++;
        //                    information.images[i].Eo.Point.Z -= Solution[information.images[i].ImageID * image_params + tx]; tx++;
        //                    information.images[i].Eo.OPK.Omega -= Solution[information.images[i].ImageID * image_params + tx]; tx++;
        //                    information.images[i].Eo.OPK.Phi   -= Solution[information.images[i].ImageID * image_params + tx]; tx++;
        //                    information.images[i].Eo.OPK.Kappa -= Solution[information.images[i].ImageID * image_params + tx]; tx++;
        //                    if (information.detectdistortion)
        //                    {
        //                        if (information.images[i].Camera.m_DistortionType == SpacecraftPlatform::CAMERA::CameraDistortionType::Classical)
        //                        {
        //                            Solution[information.image_counts * image_params + gcps + 1] = Solution[information.image_counts * image_params + gcps + 1] /
        //                                    (information.images[i].Camera.pixelsize / 1000);//x

        //                            tx = 0;
        //                            information.images[i].Camera.focus -= Solution[information.image_counts * image_params + gcps + tx] * 1000; tx++;
        //                            double dx = -Solution[information.image_counts * image_params + gcps + tx] * 1000; tx++;
        //                            double dy = 0;//-Solution[information.image_counts * image_params + gcps + tx] * 1000; tx++;
        //                            if (information.images[i].Camera.x_direction == SpacecraftPlatform::CAMERA::CameraXdirection::right)		///y == top
        //                            {
        //                                information.images[i].Camera.sample += dx;
        //                                information.images[i].Camera.line += -dy;
        //                            }
        //                            else if (information.images[i].Camera.x_direction == SpacecraftPlatform::CAMERA::CameraXdirection::top)	///y == left
        //                            {
        //                                information.images[i].Camera.line += -dx;
        //                                information.images[i].Camera.sample += -dy;
        //                            }
        //                            else if (information.images[i].Camera.x_direction == SpacecraftPlatform::CAMERA::CameraXdirection::left)	///y == bottom
        //                            {
        //                                information.images[i].Camera.sample += -dx;
        //                                information.images[i].Camera.line += dy;
        //                            }
        //                            else if (information.images[i].Camera.x_direction == SpacecraftPlatform::CAMERA::CameraXdirection::bottom)	///y == right
        //                            {
        //                                information.images[i].Camera.line += dx;
        //                                information.images[i].Camera.sample += dy;
        //                            }
        //                            information.images[i].Camera.D_K1   -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            information.images[i].Camera.D_K2   -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            information.images[i].Camera.D_K3   -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            information.images[i].Camera.D_P1   -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            information.images[i].Camera.D_P2   -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            //information.images[i].Camera.D_b1   -= Solution[information.image_counts * image_params + gcps + tx] / 1000 *0; tx++;
        //                            //information.images[i].Camera.D_b2   -= Solution[information.image_counts * image_params + gcps + tx] / 1000 *0; tx++;
        //                        }
        //                        else
        //                        {
        //                            //=====//x + ax0 + ax1*x + ax2*y + ax3*x2 + ax4*xy + ax5*y2 + ax6*x3 + ax7*x2y + ax8*xy2 + ax9*y3 = -f * X/Z
        //                            information.images[i].Camera.focus -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            information.images[i].Camera.D_ax0 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            //information.images[i].Camera.D_ax1 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            //information.images[i].Camera.D_ax2 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            information.images[i].Camera.D_ax3 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            information.images[i].Camera.D_ax4 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            information.images[i].Camera.D_ax5 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            information.images[i].Camera.D_ax6 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            if (!use_5_params)
        //                            {
        //                                information.images[i].Camera.D_ax7 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                                information.images[i].Camera.D_ay8 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                                information.images[i].Camera.D_ay9 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            }
        //                            information.images[i].Camera.D_ay0 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            //information.images[i].Camera.D_ay1 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            //information.images[i].Camera.D_ay2 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            information.images[i].Camera.D_ay3 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            information.images[i].Camera.D_ay4 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            information.images[i].Camera.D_ay5 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            information.images[i].Camera.D_ay6 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            if (!use_5_params)
        //                            {
        //                                information.images[i].Camera.D_ay7 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                                information.images[i].Camera.D_ay8 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                                information.images[i].Camera.D_ay9 -= Solution[information.image_counts * image_params + gcps + tx]; tx++;
        //                            }
        //                        }
        //                    }
        //                }
        //                //========================================
        //                double Vect3d = GetAbsDist3d(Solution);
        //                double Vect3dL = GetAbsDist3d(L)*1000 / (information.images[0].Camera.pixelsize/1000);
        //                if (Vect3d < Precision || Vect3dL < 0.01)
        //                    break;
        //            }
        //        }


        void Solve(const QVector <double>& A, const QVector <double>& L, const QVector <double>& PL,int meas_counts, int unknowns, QVector <double>& Solution)
        {
            QVector <double> AT = MatrixTranspone(A, meas_counts, unknowns);
            QVector <double> ATP = QVector <double>(AT.size());
            for (int r = 0; r < unknowns; r++)
            {
                for (int cols = 0; cols < meas_counts; cols++)
                    ATP[r*meas_counts + cols] = AT[r*(meas_counts) + cols] * PL[cols];
            }
            QVector <double> R = MatrixMultyply(ATP, unknowns, meas_counts, A, meas_counts, unknowns);
            QVector <double> B = MatrixMultyply(ATP, unknowns, meas_counts, L, meas_counts, 1);
            LDLT_solver(R, B, unknowns, Solution);
        }
        //=======================================================================
        void Adjust(Data& information)
        {
            int nogcps = GetTotalNoGCPs(information);
            int unknowns = GetUnknownParams(information);
            if (unknowns == 0 || information.meas_counts * 2 <= unknowns)
                return;
            //==========================================
            QVector <bool> CalibDistParams = QVector <bool>(10);
            GetUnknownDistortion(information, CalibDistParams);
            //========================================
            QVector <double> Solution = QVector <double>(unknowns);
            QVector <double> Solution_prev = QVector <double>(unknowns);
            //========================================
            QVector <double> A = QVector <double>(unknowns*information.meas_counts * 2);
            QVector <double> L = QVector <double>(information.meas_counts * 2);
            QVector <double> PL = QVector <double>(information.meas_counts * 2);
            //========================================
            measProduct prod;
            Position vXYZ_m;
            //==========================================================
            /*
          for (int i = 0; i < information.meas_counts; i++)
          {
            STSL::SpacecraftPlatform::CAMERA::FromImage2Cam(information.measurements[i].meas.X, information.measurements[i].meas.Y, information.measurements[i].imageID->Camera, vXYZ_m);
            vXYZ.X += vXYZ_m.X;
            vXYZ.Y += vXYZ_m.Y;
            vXYZ.Z += vXYZ_m.Z;
          }
          vXYZ.X /= information.meas_counts;
          vXYZ.Y /= information.meas_counts;
          vXYZ.Z /= information.meas_counts;
          STSL::Position^ V2 = gcnew STSL::Position();
          V2.X = 0; V2.Y = 0; V2.Z = -1;
          double omega = STSL::Deg2Radian(STSL::AngleBetweenVectors(vXYZ, V2));
          if (information.images[0].Eo.OPK.Omega == 0)
            information.images[0].Eo.OPK.Omega = omega;
          */
            //==========================================================
            int image_params = information.rot_and_shift;

            for (int iter=0;iter < 100000; iter++)
            {
                //=================================
                int row = 0;
                double Lx, Ly;
                for (int i = 0; i < information.meas_counts; i++)
                {
                    CalcProducts(information.measurements[i], prod, Lx, Ly);
                    //===========================================================================================
                    //STSL::GetXYfromXYZ(information.measurements[i].imageID->Eo, information.measurements[i].imageID->Camera, information.measurements[i].XYZg, XY);
                    //STSL::SpacecraftPlatform::CAMERA::FromImage2Cam(XY.X, XY.Y, information.measurements[i].imageID->Camera, vXYZ);
                    SpacecraftPlatform::CAMERA::FromImage2Cam(information.measurements[i].meas.X, information.measurements[i].meas.Y, information.measurements[i].imageID->Camera, vXYZ_m);
                    //===========================================================================================
                    /*L[row] = vXYZ.X - vXYZ_m.X;
              L[row + 1] = vXYZ.Y - vXYZ_m.Y;
              */
                    L[row]     = (Lx - vXYZ_m.X);
                    L[row + 1] = (Ly - vXYZ_m.Y );
                    PL[row] = 1;// 10 / (Math::Abs(L[row] * 1e6 / information.measurements[i].imageID->Camera.pixelsize));
                    PL[row + 1] = 1;// 10 / (Math::Abs(L[row + 1] * 1e6 / information.measurements[i].imageID->Camera.pixelsize));
                    //===========================================================================================
                    if (information.measurements[i].type == pType::Tie)
                    {
                        A[row*unknowns + information.image_counts * image_params + information.measurements[i].GCPsID * 3 + 0] = -prod.dx_dXs;
                        A[row*unknowns + information.image_counts * image_params + information.measurements[i].GCPsID * 3 + 1] = -prod.dx_dYs;
                        A[row*unknowns + information.image_counts * image_params + information.measurements[i].GCPsID * 3 + 2] = -prod.dx_dZs;
                        A[(row + 1)*unknowns + information.image_counts * image_params + information.measurements[i].GCPsID * 3 + 0] = -prod.dy_dXs;
                        A[(row + 1)*unknowns + information.image_counts * image_params + information.measurements[i].GCPsID * 3 + 1] = -prod.dy_dYs;
                        A[(row + 1)*unknowns + information.image_counts * image_params + information.measurements[i].GCPsID * 3 + 2] = -prod.dy_dZs;
                    }
                    //===========================================================================================
                    {
                        int tx = 0;
                        int ty = 0;
                        //====== shift
                        if (information.UType != UnknownTypes::FixALL)
                        {
                            if (information.UType == UnknownTypes::NoFix)
                            {
                                A[row*unknowns + information.measurements[i].imageID->ImageID * image_params + tx] = prod.dx_dXs; tx++;
                                A[row*unknowns + information.measurements[i].imageID->ImageID * image_params + tx] = prod.dx_dYs; tx++;
                                A[row*unknowns + information.measurements[i].imageID->ImageID * image_params + tx] = prod.dx_dZs; tx++;
                                A[(row + 1)*unknowns + information.measurements[i].imageID->ImageID * image_params + ty] = prod.dy_dXs; ty++;
                                A[(row + 1)*unknowns + information.measurements[i].imageID->ImageID * image_params + ty] = prod.dy_dYs; ty++;
                                A[(row + 1)*unknowns + information.measurements[i].imageID->ImageID * image_params + ty] = prod.dy_dZs; ty++;
                            }

                            if (information.UType == UnknownTypes::FixXYZ || information.UType == UnknownTypes::NoFix)
                            {
                                //====== rotation
                                A[row*unknowns + information.measurements[i].imageID->ImageID * image_params + tx] = prod.dx_dOm; tx++;
                                A[row*unknowns + information.measurements[i].imageID->ImageID * image_params + tx] = prod.dx_dPh; tx++;
                                A[row*unknowns + information.measurements[i].imageID->ImageID * image_params + tx] = prod.dx_dKa; tx++;
                                A[(row + 1)*unknowns + information.measurements[i].imageID->ImageID * image_params + ty] = prod.dy_dOm; ty++;
                                A[(row + 1)*unknowns + information.measurements[i].imageID->ImageID * image_params + ty] = prod.dy_dPh; ty++;
                                A[(row + 1)*unknowns + information.measurements[i].imageID->ImageID * image_params + ty] = prod.dy_dKa; ty++;
                            }
                        }
                        //======= calibration
                        if (information.CalibParams != DistortionCalibrationParams::NoCalib)
                        {
                            tx = 0;
                            ty = 0;
                            if (information.measurements[i].imageID->Camera.m_DistortionType == SpacecraftPlatform::CAMERA::CameraDistortionType::Classical)
                            {
                                QVector <double> proizv = {prod.dx_dF,prod.dx_dx0,prod.dx_dy0,prod.dx_dK1,prod.dx_dK2,prod.dx_dK3,prod.dx_dP1,prod.dx_dP2,prod.dx_db1,prod.dx_db2,
                                                         prod.dy_dF,prod.dy_dx0,prod.dy_dy0,prod.dy_dK1,prod.dy_dK2,prod.dy_dK3,prod.dy_dP1,prod.dy_dP2};
                                for (int pp=0; pp< CalibDistParams.size(); pp++)
                                {
                                    if (CalibDistParams[pp])
                                    {
                                        //x
                                        A[row*unknowns + information.image_counts * image_params + nogcps + tx] = proizv[pp];
                                        tx++;
                                        //y
                                        if (pp + 10 < proizv.size())
                                        {
                                            A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + ty] = proizv[pp + 10];
                                            ty++;
                                        }

                                    }
                                }
                                /*
                      A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_dF; tx++;
                    A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_dx0; tx++;
                    A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_dy0; tx++;
                    A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_dK1; tx++;
                    A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_dK2; tx++;
                    A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_dK3; tx++;
                    A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_dP1; tx++;
                    A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_dP2; tx++;
                    A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_db1; tx++;
                    A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_db2; tx++;
                    //y
                    A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + ty] = prod.dy_dF; ty++;
                    A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + ty] = prod.dy_dx0; ty++;
                    A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + ty] = prod.dy_dy0; ty++;
                    A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + ty] = prod.dy_dK1; ty++;
                    A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + ty] = prod.dy_dK2; ty++;
                    A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + ty] = prod.dy_dK3; ty++;
                    A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + ty] = prod.dy_dP1; ty++;
                    A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + ty] = prod.dy_dP2; ty++;
                    */
                            }
                            else if (information.measurements[i].imageID->Camera.m_DistortionType == SpacecraftPlatform::CAMERA::CameraDistortionType::IKIStyle)
                            {
                                //x
                                A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_dF; tx++;
                                A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_ax0; tx++;
                                A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_ax1; tx++;
                                A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_ax2; tx++;
                                A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_ax3; tx++;
                                A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_ax4; tx++;
                                A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_ax5; tx++;
                                A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_ax6; tx++;
                                A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_ax7; tx++;
                                A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_ax8; tx++;
                                A[row*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dx_ax9; tx++;
                                //y
                                A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + 0] = prod.dy_dF;
                                A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dy_ax0; tx++;
                                A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dy_ax1; tx++;
                                A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dy_ax2; tx++;
                                A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dy_ax3; tx++;
                                A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dy_ax4; tx++;
                                A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dy_ax5; tx++;
                                A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dy_ax6; tx++;
                                A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dy_ax7; tx++;
                                A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dy_ax8; tx++;
                                A[(row + 1)*unknowns + information.image_counts * image_params + nogcps + tx] = prod.dy_ax9; tx++;
                            }
                        }
                    }
                    //===========================================================================================
                    row += 2;
                }
                //========================================
                Solve(A,L,PL, information.meas_counts * 2, unknowns,Solution);
                //========================================
                for (int i = 0; i < information.image_counts; i++)
                {
                    int tx = 0;
                    if (information.UType != UnknownTypes::FixALL)
                    {
                        if (information.UType == UnknownTypes::NoFix)
                        {
                            information.images[i].Eo.Point.X -= Solution[information.images[i].ImageID * image_params + tx]; tx++;
                            information.images[i].Eo.Point.Y -= Solution[information.images[i].ImageID * image_params + tx]; tx++;
                            information.images[i].Eo.Point.Z -= Solution[information.images[i].ImageID * image_params + tx]; tx++;
                        }
                        if (information.UType == UnknownTypes::FixXYZ || information.UType == UnknownTypes::NoFix)
                        {
                            information.images[i].Eo.OPK.Omega -= Solution[information.images[i].ImageID * image_params + tx]; tx++;
                            information.images[i].Eo.OPK.Phi -= Solution[information.images[i].ImageID * image_params + tx]; tx++;
                            information.images[i].Eo.OPK.Kappa -= Solution[information.images[i].ImageID * image_params + tx]; tx++;
                        }
                    }
                    if (information.CalibParams != DistortionCalibrationParams::NoCalib)
                    {
                        //x = x0 + (x - x0)(D_K1 * R ^ 2 + D_K2 * R ^ 4 + D_K3 * R ^ 6) - f * X / Z
                        if (information.images[i].Camera.m_DistortionType == SpacecraftPlatform::CAMERA::CameraDistortionType::Classical)
                        {
                            /*
                  array<double>^ Normirovka = { 1000, 1000000 / information.images[i].Camera.pixelsize,
                                                     1000000 / information.images[i].Camera.pixelsize,1,1,1,1,1,0.001,0.001 };
                  */
                            QVector <double> Normirovka = { 1e3, 1e6 / information.images[i].Camera.pixelsize, 1e6 / information.images[i].Camera.pixelsize,
                                                          1e-6,1e-12,1e-18,1e-6,1e-6,1e-3,1e-3 };
                            tx = 0;
                            for (int pp = 0; pp < CalibDistParams.size(); pp++)
                            {
                                if (CalibDistParams[pp])
                                {
                                    Solution[information.image_counts * image_params + nogcps + tx] *= Normirovka[pp]; tx++;
                                }
                            }
                            tx = 0;
                            if (CalibDistParams[0])//focus
                            {
                                information.images[i].Camera.focus -= Solution[information.image_counts * image_params + nogcps + tx]; tx++;
                            }
                            if (CalibDistParams[1])//dx
                            {
                                double dx = -Solution[information.image_counts * image_params + nogcps + tx]; tx++;
                                if (information.images[i].Camera.x_direction == SpacecraftPlatform::CAMERA::CameraXdirection::right)		///y == top
                                    information.images[i].Camera.sample += dx;
                                else if (information.images[i].Camera.x_direction == SpacecraftPlatform::CAMERA::CameraXdirection::top)	///y == left
                                    information.images[i].Camera.line += -dx;
                                else if (information.images[i].Camera.x_direction == SpacecraftPlatform::CAMERA::CameraXdirection::left)	///y == bottom
                                    information.images[i].Camera.sample += -dx;
                                else if (information.images[i].Camera.x_direction == SpacecraftPlatform::CAMERA::CameraXdirection::bottom)	///y == right
                                    information.images[i].Camera.line += dx;
                            }
                            if (CalibDistParams[2])//dx
                            {
                                double dy = -Solution[information.image_counts * image_params + nogcps + tx]; tx++;
                                if (information.images[i].Camera.x_direction == SpacecraftPlatform::CAMERA::CameraXdirection::right)		///y == top
                                    information.images[i].Camera.line += -dy;
                                else if (information.images[i].Camera.x_direction == SpacecraftPlatform::CAMERA::CameraXdirection::top)	///y == left
                                    information.images[i].Camera.sample += -dy;
                                else if (information.images[i].Camera.x_direction == SpacecraftPlatform::CAMERA::CameraXdirection::left)	///y == bottom
                                    information.images[i].Camera.line += dy;
                                else if (information.images[i].Camera.x_direction == SpacecraftPlatform::CAMERA::CameraXdirection::bottom)	///y == right
                                    information.images[i].Camera.sample += dy;
                            }
                            if (CalibDistParams[3])//k1
                            {
                                information.images[i].Camera.D_K1 -= Solution[information.image_counts * image_params + nogcps + tx]; tx++;
                            }
                            if (CalibDistParams[4])//k2
                            {
                                information.images[i].Camera.D_K2 -= Solution[information.image_counts * image_params + nogcps + tx]; tx++;
                            }
                            if (CalibDistParams[5])//k3
                            {
                                information.images[i].Camera.D_K3 -= Solution[information.image_counts * image_params + nogcps + tx]; tx++;
                            }
                            if (CalibDistParams[6])//P1
                            {
                                information.images[i].Camera.D_P1 -= Solution[information.image_counts * image_params + nogcps + tx]; tx++;
                            }
                            if (CalibDistParams[7])//P2
                            {
                                information.images[i].Camera.D_P2 -= Solution[information.image_counts * image_params + nogcps + tx]; tx++;
                            }
                            if (CalibDistParams[8])//b1
                            {
                                information.images[i].Camera.D_b1 -= Solution[information.image_counts * image_params + nogcps + tx]; tx++;
                            }
                            if (CalibDistParams[9])//b2
                            {
                                information.images[i].Camera.D_b2 -= Solution[information.image_counts * image_params + nogcps + tx]; tx++;
                            }
                            /*
                  Solution[information.image_counts * image_params + nogcps + 1] = Solution[information.image_counts * image_params + nogcps + 1] / (information.images[i].Camera.pixelsize / 1000 / 1000);//x
                  Solution[information.image_counts * image_params + nogcps + 2] = Solution[information.image_counts * image_params + nogcps + 2] / (information.images[i].Camera.pixelsize / 1000 / 1000);//y
                  tx = 0;
                  information.images[i].Camera.focus -= Solution[information.image_counts * image_params + nogcps + tx] * 1000; tx++;
                  double dx = -Solution[information.image_counts * image_params + nogcps + tx]; tx++;
                  double dy = -Solution[information.image_counts * image_params + nogcps + tx]; tx++;
                  if (information.images[i].Camera.x_direction == STSL::SpacecraftPlatform::CAMERA::CameraXdirection::right)		///y == top
                  {
                    information.images[i].Camera.sample += dx;
                    information.images[i].Camera.line += -dy;
                  }
                  else if (information.images[i].Camera.x_direction == STSL::SpacecraftPlatform::CAMERA::CameraXdirection::top)	///y == left
                  {
                    information.images[i].Camera.line += -dx;
                    information.images[i].Camera.sample += -dy;
                  }
                  else if (information.images[i].Camera.x_direction == STSL::SpacecraftPlatform::CAMERA::CameraXdirection::left)	///y == bottom
                  {
                    information.images[i].Camera.sample += -dx;
                    information.images[i].Camera.line += dy;
                  }
                  else if (information.images[i].Camera.x_direction == STSL::SpacecraftPlatform::CAMERA::CameraXdirection::bottom)	///y == right
                  {
                    information.images[i].Camera.line += dx;
                    information.images[i].Camera.sample += dy;
                  }
                  information.images[i].Camera.D_K1   -= Solution[information.image_counts * image_params + nogcps + tx]; tx++;
                  information.images[i].Camera.D_K2   -= Solution[information.image_counts * image_params + nogcps + tx]; tx++;
                  information.images[i].Camera.D_K3   -= Solution[information.image_counts * image_params + nogcps + tx]; tx++;
                  information.images[i].Camera.D_P1   -= Solution[information.image_counts * image_params + nogcps + tx]; tx++;
                  information.images[i].Camera.D_P2   -= Solution[information.image_counts * image_params + nogcps + tx]; tx++;
                  information.images[i].Camera.D_b1   -= Solution[information.image_counts * image_params + nogcps + tx] / 1000; tx++;
                  information.images[i].Camera.D_b2   -= Solution[information.image_counts * image_params + nogcps + tx] / 1000; tx++;
                  */
                        }
                        else
                        {
                            //=====//x + ax0 + ax1*x + ax2*y + ax3*x2 + ax4*xy + ax5*y2 + ax6*x3 + ax7*x2y + ax8*xy2 + ax9*y3 = -f * X/Z
                            information.images[i].Camera.focus -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ax0 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ax1 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ax2 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ax3 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ax4 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ax5 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ax6 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ax7 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ax8 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ax9 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;

                            information.images[i].Camera.D_ay0 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ay1 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ay2 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ay3 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ay4 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ay5 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ay6 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ay7 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ay8 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;
                            information.images[i].Camera.D_ay9 -= Solution[information.images[i].ImageID * image_params + nogcps + tx]; tx++;

                            SpacecraftPlatform::CAMERA::CameraParams Camera_tmp;
                            Camera_tmp = information.images[i].Camera;
                            SpacecraftPlatform::CAMERA::CalcInvDistortion(Camera_tmp, 10);
                            information.images[i].Camera = Camera_tmp;
                        }
                    }
                }
                //========================================
                if (iter > 0)
                {
                    for (int i = 0; i < unknowns; i++)
                        Solution_prev[i] = std::abs(Solution_prev[i]- Solution[i]);
                }
                else
                    Solution_prev = Solution;
                //======================================== pix errors
                for (int l_d=0;l_d<L.size();l_d++)
                    L[l_d] *= 1e6 / information.images[0].Camera.pixelsize;
                //========================================
                double Vect3d = GetAbsDist3d(Solution_prev);
                double Vect3dL = GetAbsDist3d(L);//pix
                if (Vect3d < Precision || Vect3dL < 0.01)
                    break;
                //========================================
                Solution_prev = Solution;
            }
            //========================================

        }
        //=======================================================================

    };


};




