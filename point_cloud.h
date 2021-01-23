#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H

#include <library/camera.h>
#include <library/algorithms.h>

class Point_cloud : public Algorithms
{
public:
    std::vector<cv::Matx31f> projection_point(std::vector<cv::Point2f> points,cv::Mat);
    std::vector<cv::Matx31f> laser_plane_intersection(std::vector<cv::Matx31f> points, cv::Mat n, float d);
    std::vector<cv::Matx31f> homogeneus_transformation(std::vector<cv::Matx31f> points, cv::Mat R, cv::Mat T);
    std::vector<cv::Point3f> world_rotation(std::vector<cv::Matx31f> points,float X);
    std::vector<cv::Point3f> _pointcloud(std::vector<cv::Point2f>,cv::Mat,cv::Mat,float,cv::Mat,cv::Mat,float X);
    std::vector<cv::Point3f> _pointcloud_laser(std::vector<cv::Point2f>,cv::Mat,cv::Mat,float);

private:

};

#endif // POINT_CLOUD_H
