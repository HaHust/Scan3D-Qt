#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

class Camera
{
public:
    Camera();
    cv::Mat intrinsic_matrix;
    cv::Mat distortion_coefficients;
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;
    cv::VideoCapture cap;
    cv::Size board_size;
    uchar square_width;
    double fy{0},fx{0},cx{0},cy{0};



    // ---------- laser ----------
    std::vector<cv::Point3f> points_cloud_laser;
    cv::Point3f center;
    cv::Vec3f nml;
    cv::Mat n;
    float d;

    // --------- turntable-------

    std::vector<cv::Point3f> points_cloud_turntable;
    cv::Mat R,T;

};

#endif // CAMERA_H
