#-------------------------------------------------
#
# Project created by QtCreator 2017-04-10T14:44:16
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = opencv_project
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        mainwindow.cpp \
    histogram1d.cpp \
    laplacian.cpp \
    linefinder.cpp \
    harrisdetector.cpp \
    cameracalibrator.cpp

HEADERS  += mainwindow.h \
    histogram1d.h \
    laplacian.h \
    linefinder.h \
    harrisdetector.h \
    cameracalibrator.h

FORMS    += mainwindow.ui

#INCLUDEPATH +=  C:\opencv-build\install\include
#LIBS += -LC:\opencv-build\install\x86\mingw\lib \
#    -lopencv_core320.dll \
#    -lopencv_highgui320.dll \
#    -lopencv_imgcodecs320.dll \
#    -lopencv_imgproc320.dll \
#    -lopencv_features2d320.dll \
#    -lopencv_calib3d320.dll
LIBS+=-lopencv_core$${OPENCV_VERSION}
LIBS+=-lopencv_imgproc$${OPENCV_VERSION}
LIBS+=-lopencv_imgcodecs$${OPENCV_VERSION}
LIBS+=-lopencv_line_descriptor$${OPENCV_VERSION}
