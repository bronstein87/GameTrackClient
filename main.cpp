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
        MainWindow w(ipPort);
        w.show();
        return a.exec();
    }

}



