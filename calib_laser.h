#ifndef CALIB_LASER_H
#define CALIB_LASER_H

#include <QThread>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <library/algorithms.h>
#include <library/camera.h>

class Calib_laser : public QThread, public Algorithms
{
    Q_OBJECT
public:
    Calib_laser();
    cv::VideoCapture *_cam;
    Camera cam_laser;
    void run();
public slots:

signals:
    void nofy5();
};

#endif // CALIB_LASER_H
