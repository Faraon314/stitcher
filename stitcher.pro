QT += core
QT -= gui

CONFIG += c++11

TARGET = stitcher
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp \
    stitcher.cpp \
    warpers.cpp \
    line.cpp \
    blenders.cpp \
    camera.cpp \
    utils_for_delete.cpp \
    utils.cpp

HEADERS += \
    stitcher.h \
    warpers.h \
    precomp.h \
    warpercreators.h \
    line.h \
    blenders.h \
    camera.h



LIBS += -L/usr/local/lib \
-lopencv_core \
-lopencv_imgproc \
-lopencv_highgui \
-lopencv_ml \
-lopencv_video \
-lopencv_features2d \
-lopencv_calib3d \
-lopencv_objdetect \
-lopencv_flann \
-lopencv_stitching \
