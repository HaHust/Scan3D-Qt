QT       += core gui\
            serialport\
            multimedia\
            opengl\
            datavisualization

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    calib.cpp \
    capture.cpp \
    exportfile.cpp \
    library/algorithms.cpp \
    library/camera.cpp \
    main.cpp \
    mainwindow.cpp \
    point_cloud.cpp

HEADERS += \
    calib.h \
    capture.h \
    exportfile.h \
    library/algorithms.h \
    library/camera.h \
    mainwindow.h \
    point_cloud.h

FORMS += \
    mainwindow.ui
    INCLUDEPATH += D:\\opencv\\build\\include

    LIBS += D:\\opencv-build\\bin\\libopencv_core320.dll
    LIBS += D:\\opencv-build\\bin\\libopencv_highgui320.dll
    LIBS += D:\\opencv-build\\bin\\libopencv_imgcodecs320.dll
    LIBS += D:\\opencv-build\\bin\\libopencv_imgproc320.dll
    LIBS += D:\\opencv-build\\bin\\libopencv_features2d320.dll
    LIBS += D:\\opencv-build\\bin\\libopencv_calib3d320.dll
    LIBS += D:\\opencv-build\\bin\\libopencv_video320.dll
    LIBS += D:\\opencv-build\\bin\\libopencv_videostab320.dll
    LIBS += D:\\opencv-build\\bin\\libopencv_videoio320.dll

LIBS += -lOpengl32
# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
