#include "calibration.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>


Calibration::Calibration()
{
}
double Calibration::Radian2Deg(double Val)
{
  return(Val*180.0 / M_PI);
}
double Calibration::Deg2Radian(double Val)
{
  return(Val* M_PI / 180.0);
}
double Calibration::AngleBetweenVectors(Position& V1, Position& V2)
{
  double angle = 0;
  double VectorSize1 = sqrt(V1.X*V1.X + V1.Y*V1.Y + V1.Z*V1.Z);
  double VectorSize2 = sqrt(V2.X*V2.X + V2.Y*V2.Y + V2.Z*V2.Z);
  double scalar = V1.X*V2.X + V1.Y*V2.Y + V1.Z*V2.Z;
  double cos_angle = scalar / (VectorSize1*VectorSize2);
  if (cos_angle <= 1. &&  cos_angle >= -1.)
    angle = Radian2Deg(acos(cos_angle));
  else
    angle = 0;
  return(angle);
}
//четверть угла
double Calibration::QuaterOfAngle(double A, double B, Calibration::Anglesdimentions VAL)
{
  //A - Числитель В - знаменатель
  double C = 0;
  if (VAL == Calibration::Anglesdimentions::FROMZEROto360)
  {
    //Определение четверти
    if (A < 0 &&  qFuzzyCompare(B, 0))
      C = M_PI*3.0 / 2.0;
    else if (A > 0 &&  qFuzzyCompare(B, 0))
      C = M_PI / 2.0;
    else if (A >= 0 &&  B > 0)
      C = atan(abs(A / B));		            //1- четверть
    else if (A >= 0 &&  B < 0)
      C = M_PI - atan(abs(A / B));		//2- четверть
    else if (A < 0 &&  B < 0)
      C = M_PI + atan(abs(A / B));		//3- четверть
    else if (A < 0 &&  B>0)
      C = 2 * M_PI - atan(abs(A / B));//4- четверть
  }
  else if (VAL == Calibration::Anglesdimentions::PLUSMINUS180)
  {
    if (A < 0 &&  qFuzzyCompare(B, 0))
      C = -M_PI / 2.0;
    else if (A > 0 &&  qFuzzyCompare(B, 0))
      C = M_PI / 2.0;
    if (A > 0 &&  B > 0)
      C = atan(A / B);	            //I part
    else if (A < 0 &&  B > 0)
      C = atan(A / B);	            //IV part
    else if (A < 0 &&  B < 0)
      C = -M_PI + atan(A / B);	//III
    else if (A > 0 &&  B < 0)
      C = M_PI + atan(A / B);	  //II
  }
  return(C);
}
//PHOTOMOD from rotation matrix 2 omega - phi - kappa
void Calibration::Matrix2OmegaPhiKappa(RotationMatrix& Matrix, RotAngles& OPK)
{
  OPK.Omega = Calibration::QuaterOfAngle(-Matrix.b3, Matrix.c3, Calibration::Anglesdimentions::PLUSMINUS180);
  OPK.Phi = asin(Matrix.a3);
  OPK.Kappa = Calibration::QuaterOfAngle(-Matrix.a2, Matrix.a1, Calibration::Anglesdimentions::PLUSMINUS180);
}
//PHOTOMOD omega - phi - kappa 2 rotation matrix
void Calibration::OmegaPhiKappa2Matrix(RotAngles& OPK, RotationMatrix& Matrix)
{
  double cos_phi = cos(OPK.Phi);
  double sin_phi = sin(OPK.Phi);
  double cos_kappa = cos(OPK.Kappa);
  double sin_kappa = sin(OPK.Kappa);
  double cos_omega = cos(OPK.Omega);
  double sin_omega = sin(OPK.Omega);

  double a1 = cos_phi * cos_kappa;
  double a2 = -cos_phi * sin_kappa;
  double a3 = sin_phi;

  double b1 = sin_omega * sin_phi*cos_kappa + cos_omega * sin_kappa;
  double b2 = -sin_omega * sin_phi*sin_kappa + cos_omega * cos_kappa;
  double b3 = -sin_omega * cos_phi;

  double c1 = -cos_omega * sin_phi*cos_kappa + sin_omega * sin_kappa;
  double c2 = cos_omega * sin_phi*sin_kappa + sin_omega * cos_kappa;
  double c3 = cos_omega * cos_phi;

  Matrix.a1 = a1;
  Matrix.a2 = a2;
  Matrix.a3 = a3;

  Matrix.b1 = b1;
  Matrix.b2 = b2;
  Matrix.b3 = b3;

  Matrix.c1 = c1;
  Matrix.c2 = c2;
  Matrix.c3 = c3;
}
bool Calibration::SpacecraftPlatform::CAMERA:: FromImage2Cam(double sample, double line, CAMERA::CameraParams& Camera, Position& vect)
{
  //camera is a central projection
  if (Camera.cameraType == CAMERA::CameraType::central)
  {
    //=======================================================================
    if (Camera.z_direction == CAMERA::CameraZdirection::frompage)
    {
      if (Camera.x_direction == CAMERA::CameraXdirection::right)
      {
        vect.X = (sample - Camera.sample);
        vect.Y = -(line - Camera.line);
      }
      else if (Camera.x_direction == CAMERA::CameraXdirection::top)
      {
        vect.X = -(line - Camera.line);
        vect.Y = -(sample - Camera.sample);
      }
      else if (Camera.x_direction == CAMERA::CameraXdirection::left)
      {
        vect.X = -(sample - Camera.sample);
        vect.Y = (line - Camera.line);
      }
      else if (Camera.x_direction == CAMERA::CameraXdirection::bottom)
      {
        vect.X = (line - Camera.line);
        vect.Y = (sample - Camera.sample);
      }
      vect.Z = -Camera.focus;//mm
    }
    else if (Camera.z_direction == CAMERA::CameraZdirection::intopage)
    {
      if (Camera.x_direction == CAMERA::CameraXdirection::right)
      {
        vect.X = (sample - Camera.sample);
        vect.Y = (line - Camera.line);
      }
      else if (Camera.x_direction == CAMERA::CameraXdirection::top)
      {
        vect.X = -(line - Camera.line);
        vect.Y = (sample - Camera.sample);
      }
      else if (Camera.x_direction == CAMERA::CameraXdirection::left)
      {
        vect.X = -(sample - Camera.sample);
        vect.Y = -(line - Camera.line);
      }
      else if (Camera.x_direction == CAMERA::CameraXdirection::bottom)
      {
        vect.X = (line - Camera.line);
        vect.Y = -(sample - Camera.sample);
      }
      vect.Z = Camera.focus;//mm
    }
    //======================================================================= Distortion
    double pxl_size_mm = Camera.pixelsize / 1000.0;
    if (Camera.m_DistortionType == CAMERA::CameraDistortionType::Classical)
    {
      double x = vect.X * pxl_size_mm;//mm
      double y = vect.Y * pxl_size_mm;//mm
      double r = sqrt(x*x + y * y);//mm
      vect.X = x * (1.0 -
        Camera.D_K1 * pow(r, 2) -
        Camera.D_K2 * pow(r, 4) -
        Camera.D_K3 * pow(r, 6)) -
        Camera.D_P1*(2.*x*x + r * r) -
        2.* Camera.D_P2*x*y -
        x * Camera.D_b1 -
        y * Camera.D_b2;
      vect.Y = y * (1.0 -
        Camera.D_K1 * pow(r, 2) -
        Camera.D_K2 * pow(r, 4) -
        Camera.D_K3 * pow(r, 6)) -
        Camera.D_P2*(2.*y*y + r * r) -
        2.* Camera.D_P1*x*y -
        x * Camera.D_a1 -
        y * Camera.D_a2;
      //================================================================================
      vect.X = vect.X / 1000.0;//m
      vect.Y = vect.Y / 1000.0;//m
      vect.Z = vect.Z / 1000.0;//m
    }
    else if (Camera.m_DistortionType == CAMERA::CameraDistortionType::IKIStyle)
    {
      double px2mm = pxl_size_mm * vect.X;
      double py2mm = pxl_size_mm * vect.Y;
      double x2 = px2mm* px2mm;
      double x3 = px2mm * x2;
      double y2 = py2mm * py2mm;
      double y3 = py2mm * y2;
      double dx = Camera.D_ax0 + Camera.D_ax1*px2mm + Camera.D_ax2*py2mm + Camera.D_ax3*x2+
                  Camera.D_ax4*px2mm*py2mm + Camera.D_ax5*y2 + Camera.D_ax6*x3 +
                  Camera.D_ax7*x2*py2mm + Camera.D_ax8*px2mm*y2 + Camera.D_ax9*y3;
      double dy = Camera.D_ay0 + Camera.D_ay1*px2mm + Camera.D_ay2*py2mm + Camera.D_ay3*x2 +
                  Camera.D_ay4*px2mm*py2mm + Camera.D_ay5*y2 + Camera.D_ay6*x3 +
                  Camera.D_ay7*x2*py2mm + Camera.D_ay8*px2mm*y2 + Camera.D_ay9*y3;
      //================================================================================
      vect.X = (px2mm-dx) / 1000.0;//m
      vect.Y = (py2mm-dy) / 1000.0;//m
      vect.Z = vect.Z / 1000.0;//m
    }
    return(true);
  }
  //camera is a panoramic spherical
  if (Camera.cameraType == CAMERA::CameraType::spherical)
  {
    if (Camera.z_direction == CAMERA::CameraZdirection::frompage)
    {
      if (Camera.x_direction == CAMERA::CameraXdirection::right)
      {
        vect.X = -(sample - Camera.sample);///y == top
        vect.Y = -(line - Camera.line);
        ///======= умножаем на масштабный коэффициент в градусах
        double p_size_deg = Camera.pixelsize;
        double Lon = Deg2Radian(vect.X * p_size_deg);//панорамическая долгота Гамма
        double Lat = Deg2Radian(vect.Y * p_size_deg);//панорамическая широта Бетта
      ///=======
        if (Lon > M_PI)
          Lon = Lon - 2.*M_PI;
        if (Lon < -M_PI)
          Lon += 2.*M_PI;
      ///=======
        /* описание:
        система координат камеры сооветствует:
            х  - направлена на запад  (вправо),
            у - направлена на север (вверх),
            z - направлена вдоль нормали,
            минус перед y - приводит к правой ск
        */
        vect.X = cos(Lon)*cos(Lat);
        vect.Y = sin(Lon)*cos(Lat);
        vect.Z = sin(Lat);
        return(true);
      }
    }
  }
  //camera is a panoramic Cylindryc
  if (Camera.cameraType == CAMERA::CameraType::cylindrical)
  {
    if (Camera.z_direction == CAMERA::CameraZdirection::frompage)
    {
      if (Camera.x_direction == CAMERA::CameraXdirection::right)
      {
        vect.X = (sample - Camera.sample);///y == top
        vect.Y = -(line - Camera.line);
        ///======= умножаем на масштабный коэффициент в градусах
        double p_size_rad = Camera.pixelsize / 180.0*M_PI;
        double Lon = vect.X * p_size_rad;//панорамическая долгота
        double Lat = atan(vect.Y * tan(p_size_rad));
        ///=======
        vect.X = sin(Lon);
        vect.Y = cos(Lon);
        vect.Z = tan(Lat);
        /* описание:
        система координат камеры сооветствует:
            х  - направлена на запад  (вправо),
            у - направлена на север (вверх),
            z - направлена вдоль нормали,
            минус перед y - приводит к правой ск
        */
        return(true);
      }
    }
  }
  return(false);
}
//from camera to pix_image
bool Calibration::SpacecraftPlatform::CAMERA::FromCam2Image(Position& vect, CAMERA::CameraParams& Camera, double& sample, double& line)
  {
    //camera is a panoramic spherical
    if (Camera.cameraType == CAMERA::CameraType::spherical)
    {
      if (Camera.z_direction == CAMERA::CameraZdirection::frompage)
      {
        if (Camera.x_direction == CAMERA::CameraXdirection::right)		///y == top
        {
          //======= pixelsize ===== должен быть
          double pixsize = Camera.pixelsize;
          //панорамическая долгота Гамма
          double Lon = Radian2Deg(QuaterOfAngle(vect.Y, vect.X, Calibration::Anglesdimentions::PLUSMINUS180));
          //панорамическая широта Бетта
          double Lat = Radian2Deg(atan(vect.Z / sqrt(vect.X*vect.X + vect.Y*vect.Y)));
          ///=======
          vect.X = Lon / pixsize;
          vect.Y = Lat / pixsize;
          ///=======
          sample = Camera.sample - vect.X;
          line   = Camera.line   - vect.Y;
          if(line > Camera.lines || line < 0 || sample > Camera.samples || sample < 0)
            return(false);
          else
            return(true);
        }
      }
    }
    //camera is a panoramic Cylindryc
    if (Camera.cameraType == CAMERA::CameraType::cylindrical)
    {
      if (Camera.z_direction == CAMERA::CameraZdirection::frompage)
      {
        if (Camera.x_direction == CAMERA::CameraXdirection::right)
        {
          double pixsize = Camera.pixelsize;
          //панорамическая долгота Гамма
          double Lon = Calibration::QuaterOfAngle(vect.X, vect.Y, Calibration::Anglesdimentions::PLUSMINUS180);
          /*
          double Lon = atan(vect.X / vect.Y)*180.0 / M_PI;
          if (vect.X > 0 &  vect.Y > 0)//I quater
            Lon = atan(vect.X / vect.Y)*180.0 / M_PI;
          else if (vect.X < 0 &  vect.Y > 0)//IV quater
            Lon = atan(vect.X / vect.Y)*180.0 / M_PI;
          else if (vect.X > 0 &  vect.Y < 0)//II quater
            Lon = 180.0 + atan(vect.X / vect.Y)*180.0 / M_PI;
          else if (vect.X < 0 &  vect.Y < 0)//III quater
            Lon = 180.0 + atan(vect.X / vect.Y)*180.0 / M_PI;
          if (Lon > 180.0)
            Lon = Lon - 360.0;
          */
          //панорамическая широта Бетта
          double Lat = atan(vect.Z / sqrt(vect.X*vect.X + vect.Y*vect.Y));
          ///=======
          vect.X = Lon / pixsize;
          vect.Y = tan(Lat) / tan(pixsize / 180.0*M_PI);
          ///=======
          sample = vect.X + Camera.sample;
          line = -vect.Y + Camera.line;
          if (line > Camera.lines || line < 0 || sample > Camera.samples || sample < 0)
            return(false);
          else
            return(true);
        }
      }
    }
    //camera is a central projection
    if (Camera.cameraType == CAMERA::CameraType::central)
    {
      double focus = Camera.focus / 1000.0;//m
      double pixsize = double(Camera.pixelsize) / 1000000.0;//m
      if (Camera.z_direction == CAMERA::CameraZdirection::frompage)
      {
        if (vect.Z > 0)
          return false;
        //normirovanie
        vect.X = vect.X / vect.Z*-focus * 1000;//mm
        vect.Y = vect.Y / vect.Z*-focus * 1000;//mm
        vect.Z = -focus * 1000;
      //============================= distortion skip border effect
        if (Camera.m_Lims_return.Xmax > Camera.m_Lims_return.Xmin)
        {
          double pxl_size_mm = Camera.pixelsize / 1000.0;
          int mask = 100;
          if (vect.X > (Camera.m_Lims_return.Xmax- mask) * pxl_size_mm || vect.X < (Camera.m_Lims_return.Xmin - mask) * pxl_size_mm)
            return false;
          if (vect.Y > (Camera.m_Lims_return.Ymax- mask) * pxl_size_mm || vect.Y < (Camera.m_Lims_return.Ymin - mask) * pxl_size_mm)
            return false;
          //=====
          double maxR = (std::max(Camera.m_Lims_return.Xmax, Camera.m_Lims_return.Ymax) - 100)* pxl_size_mm;
          double vectMax = sqrt(vect.X*vect.X + vect.Y*vect.Y);
          if (vectMax > maxR)
            return false;
        }
        else
        {
          if (vect.X > Camera.sample*1.1 * pixsize * 1000 || vect.X < -Camera.sample*1.1 * pixsize * 1000)
            return false;
          if (vect.Y > Camera.line*1.1 * pixsize * 1000 || vect.Y < -Camera.line*1.1 * pixsize * 1000)
            return false;
        }
      //distortion return
        if (Camera.m_DistortionType == CAMERA::CameraDistortionType::Classical)
        {
          double r = sqrt(pow(vect.X, 2) + pow(vect.Y, 2));//mm *pixsize/1000.0
          vect.X = vect.X*(1.0 +
            Camera.D_K1 * pow(r, 2) +	//pix
            Camera.D_K2 * pow(r, 4) +
            Camera.D_K3 * pow(r, 6)) +
            vect.X*Camera.D_b1 +
            vect.Y*Camera.D_b2 -
            Camera.D_P1*(2.0*pow(vect.X, 2) + pow(r, 2)) -
            2.0*Camera.D_P2*vect.X*vect.Y;
          vect.Y = vect.Y*(1.0 +
            Camera.D_K1 * pow(r, 2) +	//pix
            Camera.D_K2 * pow(r, 4) +
            Camera.D_K3 * pow(r, 6)) +
            vect.X*Camera.D_a1 +
            vect.Y*Camera.D_a2 -
            Camera.D_P2*(2.0*pow(vect.Y, 2) + pow(r, 2)) -
            2.0*Camera.D_P1*vect.X*vect.Y;

        }
        else if (Camera.m_DistortionType == CAMERA::CameraDistortionType::IKIStyle)
        {
          double px2mm = vect.X;
          double py2mm = vect.Y;
          double dx = 0;
          double dy = 0;
          double x2 = px2mm * px2mm;
          double x3 = px2mm * x2;
          double y2 = py2mm * py2mm;
          double y3 = py2mm * y2;
          dx = Camera.D_Inv_ax0 + Camera.D_Inv_ax1*px2mm + Camera.D_Inv_ax2*py2mm + Camera.D_Inv_ax3*x2 +
               Camera.D_Inv_ax4*px2mm*py2mm + Camera.D_Inv_ax5*y2 + Camera.D_Inv_ax6*x3 +
               Camera.D_Inv_ax7*x2*py2mm + Camera.D_Inv_ax8*px2mm*y2 + Camera.D_Inv_ax9*y3;
          dy = Camera.D_Inv_ay0 + Camera.D_Inv_ay1*px2mm + Camera.D_Inv_ay2*py2mm + Camera.D_Inv_ay3*x2 +
               Camera.D_Inv_ay4*px2mm*py2mm + Camera.D_Inv_ay5*y2 + Camera.D_Inv_ay6*x3 +
               Camera.D_Inv_ay7*x2*py2mm + Camera.D_Inv_ay8*px2mm*y2 + Camera.D_Inv_ay9*y3;
          vect.X = vect.X + dx;
          vect.Y = vect.Y + dy;
        }
        //
        vect.X = vect.X / pixsize / 1000.;
        vect.Y = vect.Y / pixsize / 1000.;
        vect.Z = vect.Z / pixsize / 1000.;
        //
        if (Camera.x_direction == CAMERA::CameraXdirection::right)		///y == top
        {
          sample = vect.X + Camera.sample;
          line = -vect.Y + Camera.line;
        }
        else if (Camera.x_direction == CAMERA::CameraXdirection::top)	///y == left
        {
          line = -vect.X + Camera.line;
          sample = -vect.Y + Camera.sample;
        }
        else if (Camera.x_direction == CAMERA::CameraXdirection::left)	///y == bottom
        {
          sample = -vect.X + Camera.sample;
          line = vect.Y + Camera.line;
        }
        else if (Camera.x_direction == CAMERA::CameraXdirection::bottom)	///y == right
        {
          line = vect.X + Camera.line;
          sample = vect.Y + Camera.sample;
        }
      }
      else if (Camera.z_direction == CAMERA::CameraZdirection::intopage)
      {
        if (vect.Z < 0)
          return false;
        //normirovanie
        vect.X = vect.X / vect.Z*focus / pixsize;
        vect.Y = vect.Y / vect.Z*focus / pixsize;
        vect.Z = focus / pixsize;
        //distortion return
        vect.X = vect.X*(1.0 +
          Camera.D_K1 * pow(vect.X, 2) +	//pix
          Camera.D_K2 * pow(vect.X, 4) +
          Camera.D_K3 * pow(vect.X, 6)) +
          vect.X*Camera.D_b1 +
          vect.Y*Camera.D_b2;
        vect.Y = vect.Y*(1.0 +
          Camera.D_K1 * pow(vect.Y, 2) +	//pix
          Camera.D_K2 * pow(vect.Y, 4) +
          Camera.D_K3 * pow(vect.Y, 6)) +
          vect.X*Camera.D_a1 +
          vect.Y*Camera.D_a2;
        //
        if (Camera.x_direction == CAMERA::CameraXdirection::right)	///y == bottom
        {
          sample = vect.X + Camera.sample;
          line = vect.Y + Camera.line;
        }
        else if (Camera.x_direction == CAMERA::CameraXdirection::top)	///y == right
        {
          line = -vect.X + Camera.line;
          sample = vect.Y + Camera.sample;
        }
        else if (Camera.x_direction == CAMERA::CameraXdirection::left)	///y == top
        {
          sample = -vect.X + Camera.sample;
          line = -vect.Y + Camera.line;
        }
        else if (Camera.x_direction == CAMERA::CameraXdirection::bottom)//y.left
        {
          line = vect.X + Camera.line;
          sample = -vect.Y + Camera.sample;
        }
      }
      //=================================================================================
      if (line > Camera.lines || line < 0 || sample > Camera.samples || sample < 0)
        return(false);
      else
        return(true);
    }
    return(false);
  }
void Calibration::LDLT_solver(QVector<double>& R, QVector<double>& B, int N, QVector<double>& Solution)//void Adjustment::LDLT_factor(Adjustment::MatrOptimize& R, QVector<double>& B, int N, QVector<double>& X)
{
    QVector<double> L(N*N);
    QVector<double> D (N);
    double Small_value = 1e-25;
    double sum = 0;
    for (int i = 0; i < N; i++)//столбцы
    {
      for (int j = i; j < N; j++)//строки
      {
        sum = R[j*N + i];//значение вычисляемого элемента
        for (int k = 0; k < i; k++)//вычитание элементов строки из вычисляемого элемента
          sum = sum - L[i*N + k] * D[k] * L[j*N + k];
        //===================================================================
        if (qFuzzyCompare(D[i], 0))
          D[i] = Small_value;
        if (i == j)
        {
          D[i] = sum;//диагональный элемент
          L[i*N + i] = 1.;//диагональ
        }
        else
          L[j*N + i] = sum / D[i];//внедиагональный элемент
      }
    }
    QVector<double> Y(N);
    for (int i = 0; i < N; i++)
    {
      Y[i] = B[i];
      for (int j = 0; j < i; j++)
        Y[i] -= Y[j] * L[i*N + j];
    }
    //=================================
    Solution = QVector<double> (N);
    for (int i = N - 1; i >= 0; i--)
    {
      Solution[i] = Y[i] / D[i];
      for (int j = N - 1; j > i; j--)
        Solution[i] -= Solution[j] * L[j*N + i];
    }

  }
//approximate Inverse distortion
bool Calibration::SpacecraftPlatform::CAMERA::CalcInvDistortion(CAMERA::CameraParams& Camera, int step)
{
  int rows = Camera.lines;
  int cols = Camera.samples;
  Position vect_corr;
  Position vect;
  QVector<double> ATA (10, 0);
  QVector<double> ATL_s(10, 0);
  QVector<double> ATL_l(10, 0);

  double pxl_size_mm = Camera.pixelsize / 1000.0;
  Limits lims_mm;
  lims_mm.Xmax = 0; lims_mm.Ymax = 0;
  lims_mm.Xmin = 0; lims_mm.Ymin = 0;
  CAMERA::CameraParams CamNoDistortion;
  CamNoDistortion.Init(Camera.name, Camera.focus, Camera.pixelsize, Camera.sample, Camera.line, Camera.samples, Camera.lines, Camera.x_direction, Camera.z_direction, Camera.cameraType, Camera.m_DistortionType);
  //================================================================================
  for (int i = 0; i < rows; i+=step)
  {
    for (int j = 0; j < cols; j+=step)
    {
      Calibration::SpacecraftPlatform::CAMERA::FromImage2Cam((double)j, (double)i, Camera, vect_corr);
      Calibration::SpacecraftPlatform::CAMERA::FromImage2Cam((double)j, (double)i, CamNoDistortion, vect);
      //================================================================================
      double dx = (vect.X - vect_corr.X)*1000.;
      double dy = (vect.Y - vect_corr.Y)*1000.;
      double px2mm_back = vect_corr.X*1000;
      double py2mm_back = vect_corr.Y*1000;
      lims_mm.Xmax = std::max(px2mm_back / pxl_size_mm, lims_mm.Xmax);
      lims_mm.Xmin = std::min(px2mm_back / pxl_size_mm, lims_mm.Xmin);
      lims_mm.Ymax = std::max(py2mm_back / pxl_size_mm, lims_mm.Ymax);
      lims_mm.Ymin = std::min(py2mm_back / pxl_size_mm, lims_mm.Ymin);
      //================================================================================
      double x2_back = px2mm_back * px2mm_back;
      double x3_back = px2mm_back * x2_back;
      double y2_back = py2mm_back * py2mm_back;
      double y3_back = py2mm_back * y2_back;
      double x2y = x2_back * py2mm_back;
      double xy2 = px2mm_back * y2_back;
      double x2y2 = x2_back * y2_back;
      double xy = px2mm_back * py2mm_back;
      int crow = 0;
      ATA[crow + 0] += 1.0;         ATA[crow + 1] += px2mm_back;            ATA[crow + 2] += py2mm_back;            ATA[crow + 3] += x2_back;             ATA[crow + 4] += xy;            ATA[crow + 5] += y2_back;             ATA[crow + 6] += x3_back;             ATA[crow + 7] += x2y;               ATA[crow + 8] += xy2;               ATA[crow + 9] += y3_back;             crow += 10;
      ATA[crow + 0] += px2mm_back;  ATA[crow + 1] += x2_back;               ATA[crow + 2] += xy;                    ATA[crow + 3] += x3_back;             ATA[crow + 4] += x2y;           ATA[crow + 5] += xy2;                 ATA[crow + 6] += px2mm_back*x3_back;  ATA[crow + 7] += x2_back * xy;      ATA[crow + 8] += x2y2;              ATA[crow + 9] += px2mm_back*y3_back;  crow += 10;
      ATA[crow + 0] += py2mm_back;  ATA[crow + 1] += xy;                    ATA[crow + 2] += y2_back;               ATA[crow + 3] += x2y;                 ATA[crow + 4] += xy2;           ATA[crow + 5] += y3_back;             ATA[crow + 6] += py2mm_back*x3_back;  ATA[crow + 7] += y2_back *x2_back;  ATA[crow + 8] += xy * y2_back;      ATA[crow + 9] += y2_back *y2_back;    crow += 10;
      ATA[crow + 0] += x2_back;     ATA[crow + 1] += x3_back;               ATA[crow + 2] += x2y;                   ATA[crow + 3] += x2_back*x2_back;     ATA[crow + 4] += x2_back*xy;    ATA[crow + 5] += x2y2;                ATA[crow + 6] += x2_back*x3_back;     ATA[crow + 7] += x2_back*x2y;       ATA[crow + 8] += x3_back * y2_back; ATA[crow + 9] += x2_back*y3_back;     crow += 10;
      ATA[crow + 0] += xy;          ATA[crow + 1] += x2y;                   ATA[crow + 2] += xy2;                   ATA[crow + 3] += xy * x2_back;        ATA[crow + 4] += xy*xy;         ATA[crow + 5] += xy*y2_back;          ATA[crow + 6] += xy*x3_back;          ATA[crow + 7] += xy2*x2_back;       ATA[crow + 8] += x2y * y2_back;     ATA[crow + 9] += xy*y3_back;          crow += 10;
      ATA[crow + 0] += y2_back;     ATA[crow + 1] += xy2;                   ATA[crow + 2] += y3_back;               ATA[crow + 3] += x2y2;                ATA[crow + 4] += y2_back * xy;  ATA[crow + 5] += y2_back * y2_back;   ATA[crow + 6] += y2_back * x3_back;   ATA[crow + 7] += y2_back * x2y;     ATA[crow + 8] += y2_back * xy2;     ATA[crow + 9] += y2_back * y3_back;   crow += 10;
      ATA[crow + 0] += x3_back;     ATA[crow + 1] += x3_back * px2mm_back;  ATA[crow + 2] += x2_back * xy;          ATA[crow + 3] += x3_back * x2_back;   ATA[crow + 4] += x3_back * xy;  ATA[crow + 5] += x3_back * y2_back;   ATA[crow + 6] += x3_back * x3_back;   ATA[crow + 7] += x3_back * x2y;     ATA[crow + 8] += x3_back * xy2;     ATA[crow + 9] += x3_back * y3_back;   crow += 10;
      ATA[crow + 0] += x2y;         ATA[crow + 1] += x2y * px2mm_back;      ATA[crow + 2] += x2y2;                  ATA[crow + 3] += x2y * x2_back;       ATA[crow + 4] += x2y * xy;      ATA[crow + 5] += x2y * y2_back;       ATA[crow + 6] += x2y * x3_back;       ATA[crow + 7] += x2y * x2y;         ATA[crow + 8] += x2y * xy2;         ATA[crow + 9] += x2y * y3_back;       crow += 10;
      ATA[crow + 0] += xy2;         ATA[crow + 1] += x2y2;                  ATA[crow + 2] += xy2 * py2mm_back;      ATA[crow + 3] += xy2 * x2_back;       ATA[crow + 4] += xy2 * xy;      ATA[crow + 5] += xy2 * y2_back;       ATA[crow + 6] += xy2 * x3_back;       ATA[crow + 7] += xy2 * x2y;         ATA[crow + 8] += xy2 * xy2;         ATA[crow + 9] += xy2 * y3_back;       crow += 10;
      ATA[crow + 0] += y3_back;     ATA[crow + 1] += y3_back * px2mm_back;  ATA[crow + 2] += y3_back * py2mm_back;  ATA[crow + 3] += y3_back * x2_back;   ATA[crow + 4] += y3_back * xy;  ATA[crow + 5] += y3_back * y2_back;   ATA[crow + 6] += y3_back * x3_back;   ATA[crow + 7] += y3_back * x2y;     ATA[crow + 8] += y3_back * xy2;     ATA[crow + 9] += y3_back * y3_back;   crow += 10;
      //=============================================
      ATL_s[0] += dx;               ATL_s[1] += dx * px2mm_back;            ATL_s[2] += dx *py2mm_back;             ATL_s[3] += dx * x2_back;             ATL_s[4] += dx * xy;
      ATL_l[0] += dy;               ATL_l[1] += dy * px2mm_back;            ATL_l[2] += dy *py2mm_back;             ATL_l[3] += dy * x2_back;             ATL_l[4] += dy * xy;
      ATL_s[5] += dx * y2_back;     ATL_s[6] += dx * x3_back;               ATL_s[7] += dx * x2y;                   ATL_s[8] += dx * xy2;                 ATL_s[9] += dx * y3_back;
      ATL_l[5] += dy * y2_back;     ATL_l[6] += dy * x3_back;               ATL_l[7] += dy * x2y;                   ATL_l[8] += dy * xy2;                 ATL_l[9] += dy * y3_back;
    }
  }

  Camera.m_Lims_return.Xmax = lims_mm.Xmax;
  Camera.m_Lims_return.Ymax = lims_mm.Ymax;
  Camera.m_Lims_return.Xmin = lims_mm.Xmin;
  Camera.m_Lims_return.Ymin = lims_mm.Ymin;

  QVector<double> inv_X (10);
  Calibration::LDLT_solver(ATA, ATL_s, 10, inv_X);
  QVector<double> inv_Y (10);
  Calibration::LDLT_solver(ATA, ATL_l, 10, inv_Y);
  //==========

  //==========
  Camera.D_Inv_ax0 = inv_X[0]; Camera.D_Inv_ay0 = inv_Y[0];
  Camera.D_Inv_ax1 = inv_X[1]; Camera.D_Inv_ay1 = inv_Y[1];
  Camera.D_Inv_ax2 = inv_X[2]; Camera.D_Inv_ay2 = inv_Y[2];
  Camera.D_Inv_ax3 = inv_X[3]; Camera.D_Inv_ay3 = inv_Y[3];
  Camera.D_Inv_ax4 = inv_X[4]; Camera.D_Inv_ay4 = inv_Y[4];
  Camera.D_Inv_ax5 = inv_X[5]; Camera.D_Inv_ay5 = inv_Y[5];
  Camera.D_Inv_ax6 = inv_X[6]; Camera.D_Inv_ay6 = inv_Y[6];
  Camera.D_Inv_ax7 = inv_X[7]; Camera.D_Inv_ay7 = inv_Y[7];
  Camera.D_Inv_ax8 = inv_X[8]; Camera.D_Inv_ay8 = inv_Y[8];
  Camera.D_Inv_ax9 = inv_X[9]; Camera.D_Inv_ay9 = inv_Y[9];
  //====================================== quality approximation ======================================
  double max_error_x, max_error_y, min_error_x, min_error_y,stddev_x, stddev_y;
  max_error_x = max_error_y = min_error_x = min_error_y= stddev_x =stddev_y =0;
  double goodCounts = 0;
  for (int i = 0; i < rows; i += step)
  {
    for (int j = 0; j < cols; j += step)
    {
      double sample = j;
      double line = i;
      double sample_back = -1;
      double line_back = -1;
      Calibration::SpacecraftPlatform::CAMERA::FromImage2Cam(sample, line, Camera, vect);
      if (Calibration::SpacecraftPlatform::CAMERA::FromCam2Image(vect, Camera, sample_back, line_back))
      {
        double ds = sample_back - sample;
        double dl = line_back - line;
        max_error_x = std::max(max_error_x, ds);
        max_error_y = std::max(max_error_y, dl);
        min_error_x = std::min(min_error_x, ds);
        min_error_y = std::min(min_error_y, dl);
        stddev_x += ds * ds;
        stddev_y += dl * dl;
        goodCounts+=1.0;
      }
    }
  }
  //double dev_x = sqrt(stddev_x / goodCounts);
  //double dev_y = sqrt(stddev_y / goodCounts);
  return true;
}
//=======================================================================================================================
//from cam to inSpacecraftPlatform     ============     переход из СК камеры в СК панорамной камеры или СК аппарата
void Calibration::SpacecraftPlatform::FromCam2SpacecraftPlatform(Position& inCAM, RotationMatrix& newMatrix, Position& inSpacecraftPlatform)
{
  inSpacecraftPlatform.X = inCAM.X*newMatrix.a1 + inCAM.Y*newMatrix.a2 + inCAM.Z*newMatrix.a3;
  inSpacecraftPlatform.Y = inCAM.X*newMatrix.b1 + inCAM.Y*newMatrix.b2 + inCAM.Z*newMatrix.b3;
  inSpacecraftPlatform.Z = inCAM.X*newMatrix.c1 + inCAM.Y*newMatrix.c2 + inCAM.Z*newMatrix.c3;
}
//from inSpacecraftPlatform to cam
void Calibration::SpacecraftPlatform::FromSpacecraftPlatform2Cam(Position& inSpacecraftPlatform, RotationMatrix& newMatrix, Position& inCAM)
{
  //
  inCAM.X = inSpacecraftPlatform.X*newMatrix.a1 + inSpacecraftPlatform.Y*newMatrix.b1 + inSpacecraftPlatform.Z*newMatrix.c1;
  inCAM.Y = inSpacecraftPlatform.X*newMatrix.a2 + inSpacecraftPlatform.Y*newMatrix.b2 + inSpacecraftPlatform.Z*newMatrix.c2;
  inCAM.Z = inSpacecraftPlatform.X*newMatrix.a3 + inSpacecraftPlatform.Y*newMatrix.b3 + inSpacecraftPlatform.Z*newMatrix.c3;
}
//=======================================================================================================================
bool Calibration::VectorIntersect3D(Position& Pnt1, Position& Vect1, Position& Pnt2, Position& Vect2, Position& Result)
{

  Position Pnt3D1;
  Position Pnt3D2;
  //===================================================
  double N = ((Pnt2.Y - Pnt1.Y)*Vect1.X - (Pnt2.X - Pnt1.X)*Vect1.Y) / (Vect1.Y*Vect2.X - Vect1.X*Vect2.Y);
  //===================================================
  Pnt3D1.X = Pnt1.X + N * Vect1.X;
  Pnt3D1.Y = Pnt1.Y + N * Vect1.Y;
  Pnt3D1.Z = Pnt1.Z + N * Vect1.Z;
  //===================================================
  double N1 = ((Pnt2.X - Pnt1.X)*Vect1.Y - (Pnt2.Y - Pnt1.Y)*Vect1.X) / (Vect1.X*Vect2.Y - Vect1.Y*Vect2.X);
  //===================================================
  Pnt3D2.X = Pnt2.X + N1 * Vect2.X;
  Pnt3D2.Y = Pnt2.Y + N1 * Vect2.Y;
  Pnt3D2.Z = Pnt2.Z + N1 * Vect2.Z;
  Position Vect_1;
  Vect_1.X = Pnt3D1.X - Pnt1.X;
  Vect_1.Y = Pnt3D1.Y - Pnt1.Y;
  Vect_1.Z = Pnt3D1.Z - Pnt1.Z;
  double A1 = Calibration::AngleBetweenVectors(Vect1, Vect_1);
  Vect_1.X = Pnt3D2.X - Pnt2.X;
  Vect_1.Y = Pnt3D2.Y - Pnt2.Y;
  Vect_1.Z = Pnt3D2.Z - Pnt2.Z;
  double A2 = Calibration::AngleBetweenVectors(Vect2, Vect_1);
  if (abs(A2) > 90 || abs(A1) > 90)
    return(false);
  //нашлась точка
  Result.X = (Pnt3D1.X + Pnt3D2.X) / 2;
  Result.Y = (Pnt3D1.Y + Pnt3D2.Y) / 2;
  Result.Z = (Pnt3D1.Z + Pnt3D2.Z) / 2;
  //

  return(true);
}
//======================== вычсиление 3-х мерной точки по 2-м измеренным пиксельным координатам на 2-х изображениях
bool Calibration::GetXYZfromXYXY(ExteriorOr& EO_left, ExteriorOr& EO_right,
                           SpacecraftPlatform::CAMERA::CameraParams& Camera_left, SpacecraftPlatform::CAMERA::CameraParams& Camera_right,
                           Position2D& XYpix_left, Position2D& XYpix_right, Position& XYZ)
{
  //===========================================
  Position Vect_left;
  RotationMatrix ML;
  SpacecraftPlatform::CAMERA::FromImage2Cam(XYpix_left.X, XYpix_left.Y, Camera_left, Vect_left);
  OmegaPhiKappa2Matrix(EO_left.OPK, ML);
  Position VL3D;
  SpacecraftPlatform::FromCam2SpacecraftPlatform(Vect_left, ML, VL3D);
  //===========
  Position Vect_right;
  RotationMatrix MR;
  SpacecraftPlatform::CAMERA::FromImage2Cam(XYpix_right.X, XYpix_right.Y, Camera_right, Vect_right);
  OmegaPhiKappa2Matrix(EO_right.OPK, MR);
  Position VR3D;
  SpacecraftPlatform::FromCam2SpacecraftPlatform(Vect_right, MR, VR3D);
  //===========================================
  if (!VectorIntersect3D(EO_left.Point, VL3D, EO_right.Point, VR3D, XYZ))
    return false;
  return true;
}
bool Calibration::GetXYfromXYZ(ExteriorOr& EO, SpacecraftPlatform::CAMERA::CameraParams& Camera, Position& XYZ, Position2D& XYpix)
{
  Position Pos;
  Position V3D;
  RotationMatrix Matrix;
  double Line, Sample;
  OmegaPhiKappa2Matrix(EO.OPK, Matrix);
  //========
  Pos.X = XYZ.X - EO.Point.X;
  Pos.Y = XYZ.Y - EO.Point.Y;
  Pos.Z = XYZ.Z - EO.Point.Z;
  //========
  SpacecraftPlatform::FromSpacecraftPlatform2Cam(Pos, Matrix, V3D);
  if (SpacecraftPlatform::CAMERA::FromCam2Image(V3D, Camera, Sample, Line))
  {
    XYpix.X = Sample;
    XYpix.Y = Line;
    return true;
  }
  else
    return false;
}

bool Calibration::GetRayAndPoint(ExteriorOr& EO, SpacecraftPlatform::CAMERA::CameraParams& Camera, Position2D& XYpix, RayAndPoint& RayPoint)
{
  Position Vect;
  RotationMatrix M;
  if (SpacecraftPlatform::CAMERA::FromImage2Cam(XYpix.X, XYpix.Y, Camera, Vect))
  {
    OmegaPhiKappa2Matrix(EO.OPK, M);
    Position VL3D;
    SpacecraftPlatform::FromCam2SpacecraftPlatform(Vect, M, VL3D);
    RayPoint.Pos = EO.Point;
    RayPoint.Vect = VL3D;
    return true;
  }
  else
    return false;
}


 void Calibration::OmegaPhiKappa2Product(RotAngles& OPK, ProductType ptype, RotationMatrix& Matrix)
{
  double cos_phi = cos(OPK.Phi);
  double sin_phi = sin(OPK.Phi);
  double cos_kappa = cos(OPK.Kappa);
  double sin_kappa = sin(OPK.Kappa);
  double cos_omega = cos(OPK.Omega);
  double sin_omega = sin(OPK.Omega);

  double a1 = 0;	//double a1 = cos_phi*cos_kappa;
  if (ptype == ProductType::OMEGA)
    a1 = 0;
  else if (ptype == ProductType::PHI)
    a1 = -sin_phi * cos_kappa;
  else if (ptype == ProductType::KAPPA)
    a1 = cos_phi * -sin_kappa;
  //============================================================================
  double a2 = 0;	//double a2 = -cos_phi*sin_kappa;
  if (ptype == ProductType::OMEGA)
    a2 = 0;
  else if (ptype == ProductType::PHI)
    a2 = sin_phi * sin_kappa;
  else if (ptype == ProductType::KAPPA)
    a2 = -cos_phi * cos_kappa;
  //============================================================================
  double a3 = 0;	//double a3 = sin_phi;
  if (ptype == ProductType::OMEGA)
    a3 = 0;
  else if (ptype == ProductType::PHI)
    a3 = cos_phi;
  else if (ptype == ProductType::KAPPA)
    a3 = 0;
  //============================================================================
  double b1 = 0;	//double b1 = sin_omega*sin_phi*cos_kappa + cos_omega*sin_kappa;
  if (ptype == ProductType::OMEGA)
    b1 = cos_omega * sin_phi*cos_kappa + -sin_omega * sin_kappa;
  else if (ptype == ProductType::PHI)
    b1 = sin_omega * cos_phi*cos_kappa;
  else if (ptype == ProductType::KAPPA)
    b1 = sin_omega * sin_phi*-sin_kappa + cos_omega * cos_kappa;
  //============================================================================
  double b2 = 0;	//double b2 = -sin_omega*sin_phi*sin_kappa + cos_omega*cos_kappa;
  if (ptype == ProductType::OMEGA)
    b2 = -cos_omega * sin_phi*sin_kappa + -sin_omega * cos_kappa;
  else if (ptype == ProductType::PHI)
    b2 = -sin_omega * cos_phi*sin_kappa;
  else if (ptype == ProductType::KAPPA)
    b2 = -sin_omega * sin_phi*cos_kappa + cos_omega * -sin_kappa;
  //============================================================================
  double b3 = 0;	//double b3 = -sin_omega*cos_phi;
  if (ptype == ProductType::OMEGA)
    b3 = -cos_omega * cos_phi;
  else if (ptype == ProductType::PHI)
    b3 = -sin_omega * -sin_phi;
  else if (ptype == ProductType::KAPPA)
    b3 = 0;
  //============================================================================
  double c1 = 0;	//double c1 = -cos_omega*sin_phi*cos_kappa + sin_omega*sin_kappa;
  if (ptype == ProductType::OMEGA)
    c1 = sin_omega * sin_phi*cos_kappa + cos_omega * sin_kappa;
  else if (ptype == ProductType::PHI)
    c1 = -cos_omega * cos_phi*cos_kappa;
  else if (ptype == ProductType::KAPPA)
    c1 = -cos_omega * sin_phi*-sin_kappa + sin_omega * cos_kappa;
  //============================================================================
  double c2 = 0;	//double c2 = cos_omega*sin_phi*sin_kappa + sin_omega*cos_kappa;
  if (ptype == ProductType::OMEGA)
    c2 = -sin_omega * sin_phi*sin_kappa + cos_omega * cos_kappa;
  else if (ptype == ProductType::PHI)
    c2 = cos_omega * cos_phi*sin_kappa;
  else if (ptype == ProductType::KAPPA)
    c2 = cos_omega * sin_phi*cos_kappa + sin_omega * -sin_kappa;
  //============================================================================
  double c3 = 0;	//double c3 = cos_omega*cos_phi;
  if (ptype == ProductType::OMEGA)
    c3 = -sin_omega * cos_phi;
  else if (ptype == ProductType::PHI)
    c3 = cos_omega * -sin_phi;
  else if (ptype == ProductType::KAPPA)
    c3 = 0;
  //============================================================================
  Matrix.a1 = a1;
  Matrix.a2 = a2;
  Matrix.a3 = a3;

  Matrix.b1 = b1;
  Matrix.b2 = b2;
  Matrix.b3 = b3;

  Matrix.c1 = c1;
  Matrix.c2 = c2;
  Matrix.c3 = c3;
}

QVector <double> Calibration::MatrixTranspone(QVector <double>& InitMat, int Rows, int Cols)
{
  QVector <double> TMatrix(Rows*Cols);
  for (int i = 0; i < Rows; i++)
  {
    for (int j = 0; j < Cols; j++)
      TMatrix[j * Rows + i] = InitMat[i * Cols + j];
  }
  return(TMatrix);
}




QVector <double> Calibration::MatrixMultyply(QVector <double>& InitMat1, int Rows1, int Cols1, QVector <double>& InitMat2, int Rows2, int Cols2)
{
  if (Cols1 != Rows2)
    return QVector <double>();
  //===================================================================
  QVector <double> MultMat12(Rows1 * Cols2);
  for (int i = 0; i < Rows1; i++)
  {
    for (int j = 0; j < Cols2; j++)
    {
      for (int k = 0; k < Cols1; k++)
      {
        MultMat12[i * Cols2 + j] += InitMat1[i * Cols1 + k] * InitMat2[k * Cols2 + j];
      }
    }
  }
  return(MultMat12);
}

double Calibration::GetAbsDist3d(QVector <double>& vect)
{
  double size = 0;
  for (int i = 0; i < vect.size(); i++)
    size += vect[i] * vect[i];
  return(sqrt(size / vect.size()));
}



////                                                Example for Baseball

////=======================================================================================================================
//System::String& Calibration::StartBaseBAll()
//{
//  System::String& result = "";
//  //======================================================================================================================================
//  Calibration::SpacecraftPlatform::CAMERA::CameraXdirection Dir_R = Calibration::SpacecraftPlatform::CAMERA::CameraXdirection::right;
//  Calibration::SpacecraftPlatform::CAMERA::CameraZdirection Dir_Z = Calibration::SpacecraftPlatform::CAMERA::CameraZdirection::frompage;
//  Calibration::SpacecraftPlatform::CAMERA::CameraDistortionType DistT = Calibration::SpacecraftPlatform::CAMERA::CameraDistortionType::Classical;
//  Calibration::SpacecraftPlatform::CAMERA::CameraType CamType = Calibration::SpacecraftPlatform::CAMERA::CameraType::central;
//  //======================================================================================================================================
//  Calibration::ExteriorOr& EO_left = gcnew Calibration::ExteriorOr();
//  Calibration::ExteriorOr& EO_right = gcnew Calibration::ExteriorOr();
//  Calibration::SpacecraftPlatform::CAMERA::CameraParams& Camera_left = gcnew Calibration::SpacecraftPlatform::CAMERA::CameraParams();
//  Calibration::SpacecraftPlatform::CAMERA::CameraParams& Camera_right = gcnew Calibration::SpacecraftPlatform::CAMERA::CameraParams();
//  Calibration::Position2D& XYpix_left = gcnew Calibration::Position2D();
//  Calibration::Position2D& XYpix_right = gcnew Calibration::Position2D();
//  Calibration::Position& XYZ = gcnew Calibration::Position();
//  //===================================================================  Set EO (xyz_opk in radians)  ===================================================================
//  EO_left.Init(-17.991471, -5.003513, 4.498402,Calibration::Deg2Radian(71.565202039601), Calibration::Deg2Radian(-67.699490387514), Calibration::Deg2Radian(-17.151111672969));
//  EO_right.Init(-5.011129, -17.996709, 4.503400,Calibration::Deg2Radian(82.603784380449), Calibration::Deg2Radian(-21.118703676195), Calibration::Deg2Radian(-2.667875084203));
//  //===================================================================  Set camera   ===================================================================
//  Camera_left.Init("Камера 1", 20, 5.86, 1980 / 2, 1080 / 2, 1980, 1080, Dir_R, Dir_Z, CamType, DistT);
//  Camera_right.Init("Камера 2", 20, 5.86, 1980 / 2, 1080 / 2, 1980, 1080, Dir_R, Dir_Z, CamType, DistT);
//  //===================================================================  Set Point meas (PIX) ===================================================================
//  QVector<Calibration::Position2D&>& pos_L = gcnew QVector<Calibration::Position2D&>(4);
//  QVector<Calibration::Position2D&>& pos_R = gcnew QVector<Calibration::Position2D&>(4);
//  for (int i = 0; i < 4; i++)
//  {
//    pos_L[i] = gcnew Calibration::Position2D();
//    pos_R[i] = gcnew Calibration::Position2D();
//  }
//  pos_L[0].X = 1103.5; pos_L[0].Y = 1080 - 469.0; pos_R[0].X = 872.0; pos_R[0].Y = 1080 - 469.5;
//  pos_L[1].X = 930.4; pos_L[1].Y = 1080 - 521.0; pos_R[1].X = 1048.02; pos_R[1].Y = 1080 - 520.8;
//  pos_L[2].X = 728.8; pos_L[2].Y = 1080 - 580.8; pos_R[2].X = 1250.0; pos_R[2].Y = 1080 - 581.0;
//  pos_L[3].X = 622.8; pos_L[3].Y = 1080 - 612.0; pos_R[3].X = 1355.5; pos_R[3].Y = 1080 - 612.0;
//  for(int i=0;i<4;i++)
//  {
//    XYpix_left = pos_L[i]; XYpix_right = pos_R[i];
//    //======================================================================================================================================
//    if (Calibration::GetXYZfromXYXY(EO_left, EO_right, Camera_left, Camera_right, XYpix_left, XYpix_right, XYZ))
//    {
//      result += "\n Res: X: " + XYZ.X.ToString("#0.0000") + " Y: " + XYZ.Y.ToString("#0.0000") + " Z: " + XYZ.Z.ToString("#0.0000");
//      Calibration::Position2D& XYpix_check = gcnew Calibration::Position2D();
//      if (GetXYfromXYZ(EO_left, Camera_left, XYZ, XYpix_check))
//      {
//        result += "\n Res: delta Xpix: " + (XYpix_check.X - XYpix_left.X).ToString("#0.0000") + " delta Ypix: " + (XYpix_check.Y - XYpix_left.Y).ToString("#0.0000");
//      }
//      if (GetXYfromXYZ(EO_right, Camera_right, XYZ, XYpix_check))
//      {
//        result += "\n Res: delta Xpix: " + (XYpix_check.X - XYpix_right.X).ToString("#0.0000") + " delta Ypix: " + (XYpix_check.Y - XYpix_right.Y).ToString("#0.0000");
//      }
//    }
//  }
//  //======================================================================================================================================
//  return (result);
//}
