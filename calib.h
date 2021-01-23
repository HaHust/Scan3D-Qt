    #ifndef CALIB_H
#define CALIB_H

#include <QThread>
#include <library/camera.h>
#include <point_cloud.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

class Calib : public QThread, public Camera, public Point_cloud
{
    Q_OBJECT

public:
    explicit Calib();
    void run();
    void capture(cv::Mat&);

    bool compute_calibration();
    double compute_cam(cv::Mat &intrinsic_matrix,cv::Mat &distortion_coefficients);

    bool point_cloud_segmen(std::vector<cv::Point3f> &pts);
    void _laser(std::vector<cv::Point3f> &,int &);
    std::vector<cv::Point3f> point_cloud_turntable();
    void _turntable(std::vector<cv::Point3f> &,int& count_calib_turntable);
    cv::Mat frame;

signals:
    void begin_camera();
    void control9();
    void remain_camera(int);
    void complete_camera();


    void control5L();
    void begin_laser();
    void complete_laser();
    void transmit_point_cloud(std::vector<cv::Point3f>);
    void laser_on();
    void laser_off();


    void begin_turntable();
    void complete_turntable();
    void control_3turntable();

};
#endif // CALIB_H
