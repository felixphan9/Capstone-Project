QT       += core gui
QT       += network
QT       += serialport
QT       += widgets
QT       += concurrent

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17
CONFIG += console
# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

#INCLUDEPATH += -I/home/phuc/build/include
#LIBS += -L/home/phuc/build/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_shape -lopencv_videoio -lopencv_imgproc -lopencv_dnn

INCLUDEPATH += -I/usr/local/include
LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_shape -lopencv_videoio -lopencv_imgproc -lopencv_dnn -lopencv_aruco -lopencv_objdetect -lopencv_calib3d -lopencv_features2d
#LIBS += -L/home/phuc/build/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_shape -lopencv_videoio -lopencv_imgproc -lopencv_dnn

INCLUDEPATH += /home/phuc/visp-ws/visp-build/include
LIBS += -L/home/phuc/visp-ws/visp-build/lib -lvisp_core -lvisp_detection -lvisp_gui -lvisp_vision -lvisp_sensor

INCLUDEPATH += -I/usr/local/include/librealsense2
INCLUDEPATH += -I/usr/local/include/librealsense2gl

LIBS += -L/usr/local/lib -lrealsense2

INCLUDEPATH += /home/phuc/visp-ws/visp-build/3rd-parties/librealsense/build/include
LIBS += -L/home/phuc/visp-ws/visp-build/3rd-parties/librealsense/build/ -lrealsense2

SOURCES += \
    calibration.cpp \
    delay_timer.cpp \
    detector.cpp \
    main.cpp \
    mainwindow.cpp \
    udp.cpp \
    yrc1000micro_com.cpp \
    yrc1000micro_command.cpp

HEADERS += \
    calibration.h \
    delay_timer.h \
    detector.h \
    mainwindow.h \
    udp.h \
    yrc1000micro_com.h \
    yrc1000micro_command.h

FORMS += \
    mainwindow.ui

# The following lines tells Qmake to use pkg-config for opencv
 QT_CONFIG -= no-pkg-config
 CONFIG  += link_pkgconfig
# PKGCONFIG += opencv4

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
