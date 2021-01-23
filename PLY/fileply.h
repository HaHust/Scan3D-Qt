#ifndef FILEPLY_H
#define FILEPLY_H

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <QString>
#include <QThread>
class Fileply : public QThread
{
    Q_OBJECT
public:
    Fileply();
    void save_points_cloud(std::vector<cv::Point3f> points_cloud);
    QString header;
    void run();
};

#endif // FILEPLY_H
