#include <gst/gst.h>
#include "mainwindow.h"
#include <QApplication>
#include <QDebug>
#include <QString>


int main(int argc, char *argv[])
{
    gst_init (&argc, &argv);
    QApplication a(argc, argv);
    QCoreApplication::setOrganizationName("IKI RAN");
    QCoreApplication::setApplicationName("BaseballCameraClient");
    qDebug() << argc;
    if (argc != 3)
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
        QString ip (argv[1]);
        ip = ip.split("_").last();
        QString port (argv[2]);
        port = port.split("_").last();
        qDebug() << ip << port;
        MainWindow w(QHostAddress(ip), port.toUShort());
        w.show();
        return a.exec();
    }


}



