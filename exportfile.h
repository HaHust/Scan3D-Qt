#ifndef EXPORTFILE_H
#define EXPORTFILE_H

#include <QThread>
#include <iostream>
#include <opencv2/core.hpp>
#include <QString>
#include <QFile>
#include <QTextStream>
class Exportfile : public QThread
{
    Q_OBJECT
public:
    Exportfile(std::vector<cv::Point3f>,QString);
    void save_points_cloud(std::vector<cv::Point3f> points_cloud);
    QString header;
    std::vector<cv::Point3f> points_cloud;
    QString fileAdress;


    void run();
};

#endif // EXPORTFILE_H
