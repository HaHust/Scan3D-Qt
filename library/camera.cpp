#include "camera.h"

Camera::Camera():intrinsic_matrix(3,3,CV_32F),distortion_coefficients(1,5,CV_32F),n(1,3,CV_32F),R(3,3,CV_32F)
{
    board_size.width = 11;
    board_size.height = 9;
    square_width = 13;
}

