#ifndef ALGORITHMS_H
#define ALGORITHMS_H


#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
class Algorithms
{
public:
    explicit Algorithms();
    void detect_line_segment(cv::Mat &image, cv::Mat &background, cv::Mat &out);

    std::vector<cv::Point2f> compute_point_2d(cv::Mat &image);

    std::vector<cv::Point2f> compute_point_2d_release(cv::Mat &image);

    bool detect_corner_chessboard(cv::Mat frame_calib,cv::Size board_size,std::vector<cv::Point2f> &corners);

    void detect_pose(std::vector<cv::Point2f> cornner ,cv::Mat &,cv::Mat &,cv::Size , cv::Mat&,cv::Mat&,cv::Mat&,float&);

    void chessboard_mark(cv::Mat &image, std::vector<cv::Point2f> corners, cv::Size board_size);

    void capture_mark(cv::Mat &image,std::vector<cv::Point2i> corners);

    void _ransan(cv::Mat &image,double &a,double &b,double &c);

    void fitPlaneToSetOfPoints(std::vector<cv::Point3f> &pts, cv::Point3f &p0, cv::Vec3f &nml);

    void _fitCircle3D(std::vector<cv::Point3f> points,cv::Mat &R,cv::Mat &T);
};

#endif // ALGORITHMS_H
