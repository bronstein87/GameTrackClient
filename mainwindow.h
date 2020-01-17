#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <cameraclient.h>
#include <QMainWindow>
#include <QMessageBox>
#include <QScopedPointer>
#include <QHostAddress>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QHostAddress address = QHostAddress(), quint16 port = 0, QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_sendMessagePushButton_clicked();

    void on_connectPushButton_clicked();

    void on_controllerParamsPushButton_clicked();

private:
    Ui::MainWindow *ui;
    QScopedPointer <CameraClient> camera;
    QScopedPointer <Camera> camHard;
};

#endif // MAINWINDOW_H
