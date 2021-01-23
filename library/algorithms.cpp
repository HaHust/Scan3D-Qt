#include "algorithms.h"

#include <QDebug>
Algorithms::Algorithms()
{

}

void Algorithms::detect_line_segment(cv::Mat &image, cv::Mat &background, cv::Mat &out )
{
    cv::subtract (image,background,out);
    cv::cvtColor (out,out,CV_BGR2GRAY);
    cv::blur (out,out,cv::Size(3,3));
    cv::threshold (out,out,30,255,CV_THRESH_TOZERO);
    std::vector<cv::Point2f> points;

}

std::vector<cv::Point2f> Algorithms::compute_point_2d(cv::Mat &image)
{
    cv::threshold (image,image,30,255,CV_THRESH_TOZERO);
    double testMaxval;
    std::vector<cv::Point2f> points;
    cv::Point2f point;
    int maxIdx[2];
    for(int i=0;i<image.rows;i++){
        cv::Mat image1 = image.row (i);
        minMaxIdx(image1, 0, &testMaxval, 0, maxIdx);
        if(testMaxval==0)
            continue;
        point.x = maxIdx[1];
        point.y = i;
        points.push_back (point);
    }
    return points;
}

std::vector<cv::Point2f> Algorithms::compute_point_2d_release(cv::Mat &image)
{
//    cv::threshold (image,image,30,255,CV_THRESH_TOZERO);
//    cv::Mat temp_col(1,image.cols,CV_64F);
//    std::vector<double> u;
//    std::vector<cv::Point2f> point;
//    cv::reduce(image,temp_col,0,cv::REDUCE_SUM,CV_64F);
//    for(int i=0;i<image.cols;i++)
//    {
//        if(temp_col.at<double>(0,i) >0){
//            u.push_back (i);
//        }
//    }
//    for(auto it:u){
//        if(.at<double>(i,u)>0){

//        }
//    }
}



bool Algorithms::detect_corner_chessboard(cv::Mat frame_calib,cv::Size board_size,std::vector<cv::Point2f> &corners)
{
    //  chuyển image sang gray
    cv::cvtColor(frame_calib, frame_calib, CV_BGR2GRAY);
    //  tìm điểm góc
    bool found = cv::findChessboardCorners(frame_calib, board_size, corners,CV_CALIB_CB_FAST_CHECK);
    if(found ) {
        cv::TermCriteria criteria = cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.001 );
        cv::cornerSubPix (frame_calib,corners,cv::Size(11,11),cv::Size(-1,-1),criteria);
        return true;
    }
    else
        return false;

}

void Algorithms::chessboard_mark(cv::Mat &image, std::vector<cv::Point2f> corners,cv::Size board_size)
{
    cv::Point2i p1,p2,p3,p4;
    p1 = corners[0];
    p2 = corners[board_size.width-1];
    p3 = corners[board_size.width*board_size.height-board_size.width];
    p4 = corners[board_size.width*board_size.height-1];
    cv::Mat mask = cv::Mat::zeros(image.rows,image.cols,CV_8UC1);
    std::vector<cv::Point2i> points{p1,p3,p4,p2};
    cv::fillConvexPoly (mask,points,cv::Scalar(255,0,0));
    image.setTo(cv::Scalar(0), ~mask);
}

void Algorithms::capture_mark(cv::Mat &image, std::vector<cv::Point2i> corners)
{
    cv::Mat mask = cv::Mat::zeros(image.rows,image.cols,CV_8UC1);
    cv::fillConvexPoly (mask,corners,cv::Scalar(255,0,0));
    image.setTo(cv::Scalar(0), ~mask);
}

/*

------------------- thuật toán RANSAC -----------------------------
                  Dùng hàm _ransac() để chạy thuật toán
                      */

double uniformRandom(void)
{
    return (double)rand() / (double)RAND_MAX;
}

void calcLinePara(std::vector<cv::Point2f> pts, double &a, double &b, double &c, double &res)
{
    res = 0;
    cv::Vec4f line;
    std::vector<cv::Point2f> ptsF;
    for (unsigned int i = 0; i < pts.size(); i++)
        ptsF.push_back(pts[i]);

    fitLine(ptsF, line, CV_DIST_L2, 0, 1e-2, 1e-2);
    a = line[1];
    b = -line[0];
    c = line[0] * line[3] - line[1] * line[2];
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        double resid_ = fabs(pts[i].x * a + pts[i].y * b + c);
        res += resid_;
    }
    res /= pts.size();
}

bool verifyComposition(const std::vector<cv::Point2f> pts)
{
    cv::Point2f pt1 = pts[0];
    cv::Point2f pt2 = pts[1];
    if (abs(pt1.x - pt2.x) < 5 && abs(pt1.y - pt2.y) < 5)
        return false;

    return true;
}

bool getSample(std::vector<int> set, std::vector<int> &sset)
{
    int i[2];
    if (set.size() > 2)
    {
        do
        {
            for (int n = 0; n < 2; n++)
                i[n] = int(uniformRandom() * (set.size() - 1));
        } while (!(i[1] != i[0]));
        for (int n = 0; n < 2; n++)
        {
            sset.push_back(i[n]);
        }
    }
    else
    {
        return false;
    }
    return true;
}

void fitLineRANSAC(std::vector<cv::Point2f> ptSet, double &a, double &b, double &c, std::vector<bool> &inlierFlag)
{
    double residual_error = 1; // inner point threshold
    bool stop_loop = false;
    int maximum = 0; //maximum number of points

    //final inner point identifier and its residual
    inlierFlag = std::vector<bool>(ptSet.size(), false);
    std::vector<double> resids_(ptSet.size(), 3);
    int sample_count = 0;
    int N = 500;

    double res = 0;

    // RANSAC
    srand((unsigned int)time(NULL)); //Set random number seed
    std::vector<int> ptsID;
    for (unsigned int i = 0; i < ptSet.size(); i++)
        ptsID.push_back(i);
    while (N > sample_count && !stop_loop)
    {
        std::vector<bool> inlierstemp;
        std::vector<double> residualstemp;
        std::vector<int> ptss;
        int inlier_count = 0;
        if (!getSample(ptsID, ptss))
        {
            stop_loop = true;
            continue;
        }
        std::vector<cv::Point2f> pt_sam;
        pt_sam.push_back(ptSet[ptss[0]]);
        pt_sam.push_back(ptSet[ptss[1]]);
        if (!verifyComposition(pt_sam))
        {
            ++sample_count;
            continue;
        }
        // Calculate the line equation
        calcLinePara(pt_sam, a, b, c, res);
        //Inside point test
        for (unsigned int i = 0; i < ptSet.size(); i++)
        {
            cv::Point2f pt = ptSet[i];
            double resid_ = fabs(pt.x * a + pt.y * b + c);
            residualstemp.push_back(resid_);
            inlierstemp.push_back(false);
            if (resid_ < residual_error)
            {
                ++inlier_count;
                inlierstemp[i] = true;
            }
        }
        // find the best fit straight line
        if (inlier_count >= maximum)
        {
            maximum = inlier_count;
            resids_ = residualstemp;
            inlierFlag = inlierstemp;
        }
        // Update the number of RANSAC iterations, as well as the probability of interior points
        if (inlier_count == 0)
        {
            N = 500;
        }
        else
        {
            double epsilon = 1.0 - double(inlier_count) / (double)ptSet.size(); // wild value point scale
            double p = 0.99; //the probability of having 1 good sample in all samples
            double s = 2.0;
            N = int(log(1.0 - p) / log(1.0 - pow((1.0 - epsilon), s)));
        }
        ++sample_count;
    }
    // Use all the interior points to re-fit the line
    std::vector<cv::Point2f> pset;
    for (unsigned int i = 0; i < ptSet.size(); i++)
    {
        if (inlierFlag[i])
            pset.push_back(ptSet[i]);
    }
    calcLinePara(pset, a, b, c, res);
}

void Algorithms::_ransan(cv::Mat &image,double &a,double &b,double &c)
{
    std::vector<cv::Point2f> point = compute_point_2d(image);
    std::vector<bool> inliers;
    fitLineRANSAC(point,a,b,c,inliers);
}



/*
 *
 * ---------------- Hết thuật toán RANSAC 2D. ---------------------------
 *
 */




/*
 *  ---------------      Thuật toán xác định hướng ---------------------
*/
void Algorithms::detect_pose(std::vector<cv::Point2f> corners,cv::Mat &intrinsic_matrix,cv::Mat &distortion_coefficients,cv::Size board_size, cv::Mat &R,cv::Mat &T,cv::Mat &n,float &d){
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points;

    if(1) {
        image_points.push_back(corners);
        object_points.push_back(std::vector<cv::Point3f>());
        std::vector<cv::Point3f>& opts = object_points.back();
        opts.resize(board_size.width*board_size.height);
        for (int j=0; j<board_size.width*board_size.height; j++) {
            opts[j] = cv::Point3d(static_cast<float>(j%board_size.width)*13
                                  ,static_cast<float>(j/board_size.width)*13
                                  ,static_cast<float>(0));
        }
    }
    cv::Mat rvec,tvec;
    cv::solvePnP (object_points[0],image_points[0],intrinsic_matrix,distortion_coefficients,rvec,tvec);

    cv::Rodrigues (rvec,R);
    // Transtion
    cv::transpose (tvec,T);
    // normal  vector
    cv::transpose (R.col (2),n);
    // distance
    d = n.dot (T);
}




/*
 *
 *
 * ----------------------------Calib Ban xoay -----------------------
 *
 *
 *
 */
void Algorithms::fitPlaneToSetOfPoints(std::vector<cv::Point3f> &pts, cv::Point3f &p0, cv::Vec3f &nml)
{
    const int SCALAR_TYPE = CV_32F;
    typedef float ScalarType;

    p0 = cv::Point3f(0,0,0);
    for (int i = 0; i < (int)pts.size(); ++i)
        p0 = p0 + (pts[i]);
    p0 *= 1.0/pts.size();

    // Compose data matrix subtracting the centroid from each point
    cv::Mat Q((int)pts.size(), 3, SCALAR_TYPE);
    for (int i = 0; i < (int)pts.size(); ++i) {
        Q.at<ScalarType>(i,0) = pts[i].x - p0.x;
        Q.at<ScalarType>(i,1) = pts[i].y - p0.y;
        Q.at<ScalarType>(i,2) = pts[i].z - p0.z;
    }

    // Compute SVD decomposition and the Total Least Squares solution, which is the eigenvector corresponding to the least eigenvalue
    cv::SVD svd(Q, cv::SVD::MODIFY_A|cv::SVD::FULL_UV);
    nml = svd.vt.row(2);
    // Calculate the actual RMS error
}

void findRotation(cv::Mat &R,cv::Vec3f nml)
{
    cv::Matx33f R1;
    cv::Point3f v2(1,0,0);
    cv::Point3f s = v2.cross (nml)/cv::norm(v2.cross (nml));
    cv::Point3f r = nml.cross (s)/cv::norm(nml.cross (s));
    R = (cv::Mat_<double>(3,3)<<s.x   ,s.y   ,s.z,r.x   ,r.y   ,r.z,nml(0),nml(1),nml(2));
    R = R.t ();


}

void fitCircle2d(std::vector<cv::Point2f> points, int N, cv::Point2f &centerOut, double & radiusOut)
{
    cv::Mat A(N, 3, CV_64FC1, cv::Scalar::all(0));
    cv::Mat Y(N, 1, CV_64FC1, cv::Scalar::all(0));

    for (int i = 0; i < N; i++)
    {
        A.at<double>(i, 0) = 2 * points[i].x;
        A.at<double>(i, 1) = 2 * points[i].y;
        A.at<double>(i, 2) = 1;
        Y.at<double>(i, 0) = pow(points[i].x, 2) + pow(points[i].y, 2);
    }

    // X = [x0;y0;pow(r,2)-pow(x0,2), pow(y0,2)]
    cv::Mat X = (A.t()*A).inv()*A.t()*Y;

    // C =r^2 - x0^2 - y0^2
    radiusOut = X.at<double>(2, 0) + pow(X.at<double>(0, 0), 2) + pow(X.at<double>(1, 0), 2);
    if (radiusOut < 0)
    {
        centerOut.x = 0;
        centerOut.y = 0;
        radiusOut = 0;
    }
    else
    {
        centerOut.x = X.at<double>(0, 0);
        centerOut.y = X.at<double>(1, 0);
        radiusOut = sqrt(radiusOut);
    }
}

std::vector<cv::Point3f> rodrigues_rot(std::vector<cv::Point3f> pts,cv::Vec3f n0,cv::Vec3f n1){

    n0 = n0/cv::norm (n0);
    n1 = n1/cv::norm (n1);

    cv::Point3f k = n0.cross (n1);

    k = k/cv::norm(k);
    float theta = acos(n0.dot (n1));

    std::vector<cv::Point3f> p_r;

    for(auto it : pts){
        cv::Point3f temp;
        temp = (it*cos(theta) + k.cross (it)*sin(theta) + k*k.dot(it)*(1-cos(theta)));
        p_r.push_back (temp);
    }
    return p_r;
}


void fitCircle3d(std::vector<cv::Point3f> pts,cv::Vec3f n0,float X,cv::Mat &T){
    cv::Vec3f n1(1,0,0);
    cv::Point3f sum;
    for(auto it:pts){
        sum += it;
    }
    sum /= (int)pts.size();
    for(auto &it:pts){
        it -= sum;
    }
    std::vector<cv::Point3f> point_xyz = rodrigues_rot(pts,n0,n1);
    std::vector<cv::Point2f> point_yz;
    for(auto it:point_xyz){
        cv::Point2f temp;
        temp.x = it.y;
        temp.y = it.z;
        point_yz.push_back(temp);
    }
    cv::Point2f center2d;
    double r;

    fitCircle2d (point_yz,(int)pts.size(),center2d,r);
    cv::Point3f center1 (0,center2d.x,center2d.y);
    std::vector<cv::Point3f> center3dtemp;
    center3dtemp.push_back (center1);
    std::vector<cv::Point3f> center3d = rodrigues_rot(center3dtemp,n1,n0);
    center3d[0] += sum;
    T = (cv::Mat_<double>(3,1)<<(double)(center3d[0].x-X*n0[0]),(double)(center3d[0].y-X*n0[1]),(double)(center3d[0].z-X*n0[2]));

}



void Algorithms::_fitCircle3D(std::vector<cv::Point3f> points,cv::Mat &R,cv::Mat &T)
{
    cv::Point3f p0;
    cv::Vec3f nml_turntable;
    fitPlaneToSetOfPoints (points,p0,nml_turntable);
    findRotation(R,nml_turntable);
    cv::Matx13f trans;
    fitCircle3d(points,nml_turntable,59,T);
}



