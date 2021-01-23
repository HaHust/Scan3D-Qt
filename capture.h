#ifndef CAPTURE_H
#define CAPTURE_H

#include <QThread>
#include <point_cloud.h>
#include <library/camera.h>
class Capture : public QThread , public Point_cloud, public Camera
{
    Q_OBJECT
public:
    explicit Capture();
    void run();

    std::vector<cv::Point3f> points_final;
    std::vector<cv::Point2i> area_points;

    std::vector<cv::Point3f> capture_line_laser(int );
    void _capture(int);
signals:
    void transmit_point_cloud(std::vector<cv::Point3f>);
    void control45_capture();
    void begin_capture();
    void complete_capture();
    void remain_capture(int);

    void c_laser_on();
    void c_laser_off();
};

#endif // CAPTURE_H
