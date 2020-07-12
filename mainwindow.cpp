#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QElapsedTimer>
#include <opencv2/highgui.hpp>

MainWindow::MainWindow(const QString ipPort, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    camHard.reset(new Camera("FT232R", true, -1));
    camera.reset(new CameraClient(camHard.data(), ipPort));

    connect(camera.data(), &CameraClient::readyMessageFromClient, this, [](auto msg){qDebug() << msg;});
    qDebug() << IS_SET_TRIGGER_HI_LO << IS_SET_TRIGGER_LO_HI;
}
MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_sendMessagePushButton_clicked()
{
    camera->sendTest(ui->testMessageLineEdit->text());
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
