#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QElapsedTimer>
#include <opencv2/highgui.hpp>

MainWindow::MainWindow(QHostAddress address, quint16 port, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    camHard.reset(new Camera("", true, 3));
    camera.reset(new CameraClient(camHard.data(), address, port));

    //camHard->procImageQueue();
    //camHard->setBallRecognizeFlagDebug(0);


//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_19_20_29.avi", 4510);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850//video_19_20_29.avi", 3850);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_16_38_50.avi", 4510); // !!

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850//video_16_38_50.avi", 3850);



//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_16_50_35.avi", 4510);
//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850//video_16_50_35.avi", 3850);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_16_51_53.avi", 4510);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850//video_16_51_53.avi", 3850);



//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_17_22_13.avi", 4510);
//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850//video_17_22_13.avi", 3850);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_17_27_36.avi", 4510);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850//video_17_27_36.avi", 3850);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_17_30_54.avi", 4510); // ???


//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850//video_17_30_54.avi", 3850); // ???

//   camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_17_54_56.avi", 4510);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850//video_17_54_56.avi", 3850);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_18_11_45.avi", 4510);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850//video_18_11_45.avi", 3850);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_18_19_20.avi", 4510);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850//video_18_19_20.avi", 3850);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_18_27_07.avi", 4510);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850//video_18_27_07.avi", 3850);

/*    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_19_06_06.avi", 4510);

    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850//video_19_06_06.avi", 3850)*/;


//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_19_15_15.avi", 4510);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850//video_19_15_15.avi", 3850);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_21_53_15.avi", 4510);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850//video_21_53_15.avi", 3850);


//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_22_28_35.avi", 4510);

//    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850//video_22_28_35.avi", 3850);


    //       camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850/video_16_38_50.avi", 3850);//!!
    //       camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_15_26_46.avi", 4510);

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850/video_15_26_48.avi", 3850);//!!
    //      camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_15_26_46.avi", 4510);

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850/video_15_26_15.avi", 3850);//!!
    //      camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_15_26_15.avi", 4510);

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850/video_15_25_35.avi", 3850);//!!
    //     camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_15_25_34.avi", 4510);

    //camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850/video_15_25_18.avi", 3850);//!!
    // camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_15_25_18.avi", 4510);

    //   camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850/video_15_25_00.avi", 3850);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_15_25_00.avi", 4510);

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850/video_15_24_00.avi", 3850);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_15_23_59.avi", 4510);

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850/video_15_23_17.avi", 3850);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_15_23_16.avi", 4510);

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850/video_15_21_52.avi", 3850);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_15_21_50.avi", 4510);

    //      camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850/video_15_20_40.avi", 3850);
    //      camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_15_20_39.avi", 4510);

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850/video_15_15_58.avi", 3850);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_15_15_58.avi", 4510);

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850/video_15_19_50.avi", 3850);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_15_19_50.avi", 4510);

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850/video_15_19_21.avi", 3850);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_15_19_21.avi", 4510);

    //   camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_15_18_42.avi", 4510);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850/video_15_18_42.avi", 3850);



    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test3850/video_15_17_05.avi", 3850);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/hit/hit_test4510/video_15_17_05.avi", 4510);





    // camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/moment/4510/video_23_33_36.avi", 4510); //paste1

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:01:27_3850.avi", 3850);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:01:27_4510.avi", 4510);// pasteСАЗОНОВ

    //  camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:02:30_3850.avi", 3850);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:02:30_4510.avi", 4510); //paste




    //   camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:03:53_3850.avi", 3850);
    // camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:03:53_4510.avi", 4510); //paste

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:04:41_3850.avi", 3850);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:04:41_4510.avi", 4510); //paste

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:07:25_3850.avi", 3850);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:07:25_4510.avi", 4510); // paste

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:08:15_3850.avi", 3850);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:08:15_4510.avi", 4510); //paste

    //   camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:09:35_3850.avi", 3850);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:09:35_4510.avi", 4510);//paste

    //   camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:11:50_3850.avi", 3850);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:11:50_4510.avi", 4510); //paste

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:12:41_3850.avi", 3850); // CЛИШКОМ ПОЗДНО ПОСМОТРЕТЬ
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:12:41_4510.avi", 4510);//paste

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:13:50_3850.avi", 3850);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:13:50_4510.avi", 4510); //paste

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:14:31_3850.avi", 3850);
    //   camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:14:31_4510.avi", 4510); //paste

    //   camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:15:17_3850.avi", 3850); // высоко
    //   camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:15:17_4510.avi", 4510); // paste

    //   camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:16:06_3850.avi", 3850); //СЛИШКОМ ПОЗДНО ПОСМОТРЕТЬ
    //   camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:16:06_4510.avi", 4510); //paste



    //   camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:17:00_3850.avi", 3850);
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:17:00_4510.avi", 4510); //paste

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:17:42_3850.avi", 3850);
    //   camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:17:42_4510.avi", 4510); //0.7 corr paste

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:18:41_3850.avi", 3850); // вбок
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:18:41_4510.avi", 4510);// paste

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:20:21_3850.avi", 3850); // NOOOOOOOOOOOO
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:20:21_4510.avi", 4510);

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:21:28_3850.avi", 3850); // поздно, посмотреть
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:21:28_4510.avi", 4510); //paste

    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:22:07_3850.avi", 3850); // ОЧЕНЬ ПОЗДНО
    //    camHard->debugRecognizeWithVideoOneThread("/home/nvidia/work_video/video2019-08-04T16:22:07_4510.avi", 4510); // paste






    //camHard->debugRecognizeProc();

    //QTime t = QTime::currentTime();
    //qDebug() << "sz" << sizeof(QTime);
    //    camHard->setPixelClock(474);

    //    //camHard->setExposure(1);
    //   camHard->stopLiveVideo();
    // camHard->setTriggerMode(IS_SET_TRIGGER_HI_LO );
    //  camHard->startLiveVideo();

    //   camHard->unlockAllBuffer();
    //    camHard->procImageQueue();

    //    camHard->setFrameRate(30);
    //     camHard->setExposure(9);
    //    IS_SIZE_2D sz;
    //   sz.s32Height = 1080;
    //   sz.s32Width = 1920;
    //   // camHard->setROI(sz);


    // camHard->startLiveVideo();

    //QElapsedTimer timer;
    //timer.start();
    // camHard->procImageQueue();
    //qDebug() << "finished" << timer.elapsed();
    // camera->sendVideoInternal(-1);
    //Mat test;
    //test = imread("/home/nvidia/Downloads/build-SimpleSingleGrab-JetsonTX2-Debug/TestImage.bmp");
    //AutoExposureHandler h;
    //h.correct(test);
    //    Mat test = imread("/home/nvidia/test.png");
    //    Mat test2;
    //    cvtColor(test, test2, CV_BGR2HSV);
    //    cv::namedWindow("video");
    ////cv::namedWindow("video2");
    //    cv::imshow("video", test);
    //    //cv::imshow("video2", test);
    //    cv::waitKey(0);
    //    //shadowing(test2, 0.4 * 255, 5, 0.1);
    //    cv::imshow("video2", shadowing(test2, 0.4 * 255, 5, 0.1));
    //            cv::waitKey(0);


    connect(camera.data(), &CameraClient::readyMessageFromClient, this, [](auto msg){qDebug() << msg;});
}
MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_sendMessagePushButton_clicked()
{
    camera->sendTest(ui->testMessageLineEdit->text().toLocal8Bit());
}

void MainWindow::on_connectPushButton_clicked()
{
    camera->connectToCameraServer(QHostAddress(ui->ipLineEdit->text()), ui->portLineEdit->text().toUShort());
}

void MainWindow::on_controllerParamsPushButton_clicked()
{
    if (!ui->controllerPatternLineEdit->text().isEmpty()
            && ui->controllerPortSpinBox->value() != -1)
    {
        camHard->activateObjectiveController(ui->controllerPatternLineEdit->text(), ui->controllerPortSpinBox->value());
    }
    else
    {
        QMessageBox::warning(this, "Внимание", "Не заданы параметры контроллера.");
    }
}
