#include "calib.h"
#include <QDebug>

Calib::Calib()
{
    board_size.width = 10;
    board_size.height = 5;
    square_width = 13;
}

void Calib::capture(cv::Mat &image)
{
    cv::Mat newMatrix,image_temp;
    cap.read (image_temp);
    newMatrix = cv::getOptimalNewCameraMatrix(intrinsic_matrix,distortion_coefficients,image_temp.size(),1);
    cv::undistort (image_temp,image,intrinsic_matrix,distortion_coefficients,newMatrix);
}

/*
 *
 *
 * -------------------------------- Camera -----------------------------
 *
 *
 *
 */

bool Calib::compute_calibration()
{
    cap.read(frame);
        std::vector<cv::Point2f> corners;
        bool found{false};
        found = cv::findChessboardCorners(frame, board_size, corners,cv::CALIB_CB_ADAPTIVE_THRESH|cv::CALIB_CB_NORMALIZE_IMAGE);
        if(found) {
            image_points.push_back(corners);
            object_points.push_back(std::vector<cv::Point3f>());
            std::vector<cv::Point3f>& opts = object_points.back();
            opts.resize(board_size.width*board_size.height);
            for (int j=0; j<board_size.width*board_size.height; j++) {
                opts[j] = cv::Point3d(static_cast<float>(j/board_size.width)
                                     ,static_cast<float>(j%board_size.width)
                                     ,static_cast<float>(0));
            }
        }
        return found;
}

double Calib::compute_cam(cv::Mat &intrinsic_matrix,cv::Mat &distortion_coefficients)
{
    std::vector<cv::Mat> rvecs, tvecs;
    return cv::calibrateCamera(
        object_points,
        image_points,
        frame.size(),
        intrinsic_matrix,
        distortion_coefficients,
        rvecs,
        tvecs,
        cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT);
}


/*
 *
 *
 * ------------------------------- Laser --------------------------------
 *
 *
 */

bool intersection(cv::Point2f o1, cv::Point2f p1, cv::Point2f o2, cv::Point2f p2,cv::Point2f &r)
{
    cv::Point2f x = o2 - o1;
    cv::Point2f d1 = p1 - o1;
    cv::Point2f d2 = p2 - o2;

    double cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < /*EPS*/1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return true;
}

bool Calib::point_cloud_segmen(std::vector<cv::Point3f> &pts)
{
    cv::Mat laser_image_on,laser_image_off,laser_image;

    emit laser_on ();
    msleep (500);
    cap.read (laser_image_on);

    emit laser_off();
    msleep (500);
    cap.read (laser_image_off);

    cv::Mat R_patten, T_patten,n_patten;
    float d_patten;

    // lọc ảnh
    detect_line_segment(laser_image_on,laser_image_off,laser_image);

    // phát hiện góc
    std::vector<cv::Point2f> cornner;
    bool found = detect_corner_chessboard(laser_image_off,board_size,cornner);

    if(found){
        detect_pose(cornner,intrinsic_matrix,distortion_coefficients,board_size, R_patten, T_patten, n_patten,d_patten);

        chessboard_mark(laser_image,cornner,board_size);
        double a,b,c;
        _ransan(laser_image,a,b,c);

        cv::Point2f p1 = cornner[0];
        cv::Point2f p2 = cornner[board_size.width-1];
        cv::Point2f p3 = cornner[board_size.width*board_size.height-board_size.width];
        cv::Point2f p4 = cornner[board_size.width*board_size.height-1];
//        cv::Point2f p5(-(c+b*555)/a,555);
//        cv::Point2f p6(-(c+b*565)/a,565);
        cv::Point2f p5(-(c+b*700)/a,700);
        cv::Point2f p6(-(c+b*710)/a,710);
        cv::Point2f p,q;
        std::vector<cv::Point2f> points_segmen;;
        intersection (p1,p2,p5,p6,p);
        intersection (p3,p4,p5,p6,q);
        for(float i =(float) p.y;i<q.y;i+=10){
            cv::Point2f point_segmen(double(-b*i-c)/(double)a,i);
            points_segmen.push_back (point_segmen);
        }

        pts = _pointcloud_laser (points_segmen,intrinsic_matrix,n_patten,d_patten);
        return true;
    }
    else
        return false;
}

void Calib::_laser(std::vector<cv::Point3f> &points_cloud,int &count_calib_laser)
{
    std::vector<cv::Point3f> point_cloud;
    bool ret = point_cloud_segmen (point_cloud);
    if(ret){
        emit control5L();
        msleep(1000);
        count_calib_laser--;
        points_cloud.insert (points_cloud.end(),point_cloud.begin (),point_cloud.end ());
        if(count_calib_laser==0){
            fitPlaneToSetOfPoints (points_cloud,center,nml);
            d = nml.dot (center);
            n.at<double>(0,0)= nml.operator() (0);
            n.at<double>(0,1)= nml.operator() (1);
            n.at<double>(0,2)= nml.operator() (2);
        }
    }


}

/*
 *
 *
 *
 * -----------------------------Turntable ----------------------------------
 *
 *
 *
 */

std::vector<cv::Point3f> Calib::point_cloud_turntable()
{
    cv::Mat turntable_image;
    cap.read (turntable_image);
    // phát hiện các góc ô bàn cờ, đặc biệt góc p3 ------------------------------------------------
    std::vector<cv::Point2f> cornner;
    detect_corner_chessboard(turntable_image,board_size,cornner);
    cv::Point2f p3 = cornner[board_size.width*board_size.height-board_size.width];
    // tìm toạ độ điểm góc p3 trong hệ toạ độ camera ----------------------------------------------
    cv::Mat R_turntable, T_turntable,n_turntable;
    float d_turntable;
    detect_pose(cornner,intrinsic_matrix,distortion_coefficients,board_size, R_turntable, T_turntable, n_turntable, d_turntable);

    std::vector<cv::Point2f> points_turntable;
    points_turntable.push_back (p3);
    std::vector<cv::Point3f> p;
    p = _pointcloud_laser (points_turntable,intrinsic_matrix,n_turntable,d_turntable);
    return p;
}

void Calib::_turntable(std::vector<cv::Point3f> &points_cloud_turntable, int &count_calib_turntable)
{
    points_cloud_turntable.push_back(point_cloud_turntable ()[0]);
    count_calib_turntable--;
    if(count_calib_turntable==0){
        _fitCircle3D(points_cloud_turntable,R,T);
//        cv::Matx33f Rotation = cv::Matx33f(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
//                                           R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
//                                           R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
//        cv::Matx31f Trans = cv::Matx31f(T.at<double>(0,0),T.at<double>(1,0),T.at<double>(2,0));

    }

}

/*
 *
 *
 *
 *
 * ------------------------------- CALIB ----------------------------------
 *
 *
 *
 */

void Calib::run()
{
    int count_calib_camera =10;
    emit begin_camera ();
    msleep (2000);
    emit laser_off ();
    while (true) {
        if(compute_calibration ())
        {
            count_calib_camera--;
            emit remain_camera (count_calib_camera);
            emit control9 ();
            msleep (1500);
            compute_cam(intrinsic_matrix,distortion_coefficients);
        }
        if(count_calib_camera ==0){
            emit complete_camera();
            break;
        }
    }

    // ------------------ laser -------------------------------

    static int count_calib_laser = 24; //24
    while (true) {

        _laser (points_cloud_laser,count_calib_laser);

        emit remain_camera (count_calib_laser);
        if(count_calib_laser ==0){
            emit complete_laser();
            break;
        }
    }

    // --------------------- turntable -------------------------

    emit begin_turntable ();
    msleep(3000);

    int count_calib_turntable = 25;//25
    while(true){
        emit control_3turntable();
        msleep(1500);
        _turntable(points_cloud_turntable,count_calib_turntable);
        emit remain_camera (count_calib_turntable);
        if(count_calib_turntable ==0){
            emit complete_turntable();
            break;
        }

    }
    emit laser_off ();
}
