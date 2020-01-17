#-------------------------------------------------
#
# Project created by QtCreator 2019-05-31T10:11:30
#
#-------------------------------------------------

QT       += core gui network serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = camera_client_work_app
TEMPLATE = app
CONFIG += c++14


SOURCES += main.cpp\
        mainwindow.cpp \
    cameraclient.cpp \
    camera.cpp \
    objectivecontroller.cpp \
    imageprocext.cpp \
    ballrecognizer.cpp \
    rtspvideohandler.cpp \
    mathfunc.cpp

HEADERS  += mainwindow.h \
    cameraclient.h \
    camera.h \
    objectivecontroller.h \
    imageprocext.h \
    ballrecognizer.h \
    mainstructs.h \
    rtspvideohandler.h \
    mathfunc.h

FORMS    += mainwindow.ui

#INCLUDEPATH += \
#./ \
#../



INCLUDEPATH += $$PWD/../../../../usr/local/include/opencv4
DEPENDPATH += $$PWD/../../../../usr/local/include/opencv4

unix:!macx: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lopencv_tracking

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lopencv_core

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lopencv_cudaarithm

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lopencv_cudaimgproc

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lopencv_shape

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lopencv_highgui

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lopencv_cudaimgproc

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lopencv_imgcodecs

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lopencv_features2d

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lopencv_cudacodec

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lopencv_cudalegacy

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lopencv_videoio

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lopencv_cudafilters

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lopencv_video

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lopencv_imgproc

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lopencv_features2d

unix:!macx: LIBS += -L$$PWD/../../../../usr/local/lib/ -lopencv_xphoto

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../usr/lib/release/ -lueye_api
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../usr/lib/debug/ -lueye_api
else:unix: LIBS += -L$$PWD/../../../../../usr/lib/ -lueye_api


INCLUDEPATH += /usr/lib/aarch64-linux-gnu/gstreamer-1.0/include
INCLUDEPATH += /usr/include/glib-2.0
INCLUDEPATH += /usr/lib/aarch64-linux-gnu/glib-2.0/include
INCLUDEPATH += /usr/include/gstreamer-1.0

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/aarch64-linux-gnu/ -lgstreamer-1.0

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/aarch64-linux-gnu/ -lgobject-2.0

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/aarch64-linux-gnu/ -lglib-2.0

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/aarch64-linux-gnu/ -lgio-2.0

unix:!macx: LIBS += -L$$PWD/../../../../../usr/lib/aarch64-linux-gnu/ -lgstrtspserver-1.0


