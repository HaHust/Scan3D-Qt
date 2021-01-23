#include "point_cloud.h"
#include <QDebug>


std::vector<cv::Matx31f> Point_cloud::projection_point(std::vector<cv::Point2f> points,cv::Mat intrinsic_matrix)
{
    std::vector<cv::Matx31f> point;
    double fx = intrinsic_matrix.at<double>(0,0);
    double fy = intrinsic_matrix.at<double>(1,1);
    double cx = intrinsic_matrix.at<double>(0,2);
    double cy = intrinsic_matrix.at<double>(1,2);

    for(auto it:points){
        cv::Matx31f temp((it.x - cx)/fx,(it.y - cy)/fy,1);
        point.push_back (temp);
    }
    return point;
}

std::vector<cv::Matx31f> Point_cloud::laser_plane_intersection(std::vector<cv::Matx31f> points, cv::Mat n1, float d)
{
    cv::Matx13f n;
    n << n1.at<double>(0,0),n1.at<double>(0,1),n1.at<double>(0,2);
    std::vector<cv::Matx31f> point;
    cv::Matx31f temp;
    for(auto it:points){
        temp = (d/n.t().dot (it))*it;
        point.push_back (temp);
    }
    return point;
}

std::vector<cv::Matx31f> Point_cloud::homogeneus_transformation(std::vector<cv::Matx31f> points, cv::Mat R, cv::Mat T)
{
    cv::Matx33f Rotation = cv::Matx33f(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                                       R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                                       R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
    cv::Matx31f Trans = cv::Matx31f(T.at<double>(0,0),T.at<double>(1,0),T.at<double>(2,0));
    std::vector<cv::Matx31f> point;
    cv::Matx31f temp;
    for(auto it:points)
    {
        temp = Rotation.t()*it - Rotation.t()*Trans;
        point.push_back (temp);
    }
    return point;
}

std::vector<cv::Point3f> Point_cloud::world_rotation(std::vector<cv::Matx31f> points,float X)
{
    double theta= (double)(-X)*M_PI/180.0;

    double c = (double)cos(theta);
    double s = (double)sin(theta);
    cv::Matx33f Rz(c,-s,0,s,c,0,0,0,1);

    std::vector<cv::Point3f> point;
    cv::Point3f temp;
    for(auto it:points)
    {
        temp.x = (Rz*it).operator()(0,0);
        temp.y = (Rz*it).operator()(1,0);
        temp.z = (Rz*it).operator()(2,0);
        if(temp.z>=0){
            point.push_back (temp);
        }
    }
    return point;
}

std::vector<cv::Point3f> Point_cloud::_pointcloud(std::vector<cv::Point2f> points2d, cv::Mat intrinsic_matrix, cv::Mat n, float d, cv::Mat R, cv::Mat T, float X)
{
    std::vector<cv::Matx31f> points3 = projection_point (points2d,intrinsic_matrix);

    std::vector<cv::Matx31f> points2 = laser_plane_intersection(points3,n,d);

    std::vector<cv::Matx31f> points1 = homogeneus_transformation (points2,R,T);

    std::vector<cv::Point3f> points  = world_rotation (points1,X);

    return points;
}

std::vector<cv::Point3f> Point_cloud::_pointcloud_laser(std::vector<cv::Point2f> points2d, cv::Mat intrinsic_matrix, cv::Mat n, float d)
{
    std::vector<cv::Matx31f> points3 = projection_point (points2d,intrinsic_matrix);
    std::vector<cv::Matx31f> points2 = laser_plane_intersection(points3,n,d);
    std::vector<cv::Point3f> point;
    cv::Point3f temp;
    for(auto it:points2)
    {
        temp.x = it.operator()(0,0);
        temp.y = it.operator()(1,0);
        temp.z = it.operator()(2,0);
        point.push_back (temp);
    }
    return point;
}





