#include "capture.h"
#include <QDebug>
Capture::Capture()
{

}

std::vector<cv::Point3f> Capture::capture_line_laser(int count_capture){
    cv::Mat capture_laser_on,capture_laser_off, capture_laser;
    emit c_laser_on();
    msleep(500);
    cap.read (capture_laser_on);

    emit c_laser_off();
    msleep (500);
    cap.read (capture_laser_off);

    detect_line_segment(capture_laser_on,capture_laser_off,capture_laser);
    capture_mark(capture_laser,area_points);
    float X_temp = 320;
    std::vector<cv::Point2f> temp;
    temp = compute_point_2d(capture_laser);
    return _pointcloud (temp,intrinsic_matrix,n,d,R,T,1.125*(X_temp-(float)count_capture));
}

void Capture::_capture(int count_capture)
{
    std::vector<cv::Point3f> temp_point3d = capture_line_laser (count_capture);
    points_final.insert (points_final.end(),temp_point3d.begin (),temp_point3d.end ());
    emit transmit_point_cloud (temp_point3d);
}


void Capture::run()
{

    int count_capture = 320;
    emit begin_capture ();
    while(true){
        emit control45_capture ();
        msleep(500);
        _capture (count_capture);
        emit remain_capture(count_capture);
        count_capture--;

        if(count_capture==0){
            emit complete_capture ();
            break;
        }
    }

}



