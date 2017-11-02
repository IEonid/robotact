#-------------------------------------------------
#
# Project created by QtCreator 2016-09-16T08:27:38
#
#-------------------------------------------------

QT       += core gui
QT       += network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = RobotAct
TEMPLATE = app

INCLUDEPATH += .
INCLUDEPATH += /usr/local/include
INCLUDEPATH += /usr/local/include/opencv
INCLUDEPATH += /usr/local/include/opencv2
INCLUDEPATH += /usr/local/include/opencv2/core
INCLUDEPATH += /usr/local/include/opencv2/highgui
INCLUDEPATH += /usr/local/include/opencv2/imgproc
INCLUDEPATH += /usr/local/include/opencv2/flann
INCLUDEPATH += /usr/local/include/opencv2/photo
INCLUDEPATH += /usr/local/include/opencv2/video
INCLUDEPATH += /usr/local/include/opencv2/features2d
INCLUDEPATH += /usr/local/include/opencv2/objdetect
INCLUDEPATH += /usr/local/include/opencv2/calib3d
INCLUDEPATH += /usr/local/include/opencv2/ml
INCLUDEPATH += /usr/local/include/opencv2/contrib
INCLUDEPATH += /home/dxy/kinect/OpenNI-Bin-Dev-Linux-x64-v1.5.7.10/Include

LIBS += `pkg-config opencv --cflags --libs`
LIBS += -L/home/dxy/kinect/OpenNI-Bin-Dev-Linux-x64-v1.5.7.10/Lib/ -lnimCodecs -lnimMockNodes -lnimRecorder -lOpenNI -lOpenNI.jni

SOURCES += main.cpp\
        robotact.cpp \
    kinectcal.cpp \
    qopencvview.cpp \
    model.cpp \
    glwidget.cpp

HEADERS  += robotact.h \
    kinectcal.h \
    qopencvview.h \
    model.h \
    glwidget.h

FORMS    += robotact.ui
