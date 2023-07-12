#include <gst/gst.h>
#include <cameraclient.h>
//#include "mainwindow.h"
#include <QCoreApplication>
#include <QDebug>
#include <QString>

#include <QScopedPointer>


int main(int argc, char *argv[])
{
    qDebug() << "start" << argc;
    QCoreApplication a(argc, argv);
    qDebug() << "start2" << argc;
    gst_init (&argc, &argv);
     qDebug() << "gst inited";
    QCoreApplication::setOrganizationName("IKI RAN");
    QCoreApplication::setApplicationName("BaseballCameraClient");
    qDebug() << argc << QCoreApplication::applicationDirPath();
    if (argc != 2)
    {
        qDebug() << "Wrong parameters count!";
        for (int i = 0; i < 3; ++i)
        {
            qDebug() << argv[i];
        }
        return 0;
    }
    else
    {
        QString ipPort (argv[1]);
        QScopedPointer <Camera> camHard;
        QScopedPointer <CameraClient> camera;
        camHard.reset(new Camera("FT232R", true, -1));
        camera.reset(new CameraClient(camHard.data(), ipPort));
        //MainWindow w(ipPort);
        //w.show();
        return a.exec();
    }
}



