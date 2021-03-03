#include <iostream>
#include <iomanip>
#include <map>
#include <utility>
#include <vector>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
using namespace cv;

rs2_extrinsics get_extrinsics(const rs2::stream_profile& from_stream, 
                    const rs2::stream_profile& to_stream){
    // If the device/sensor that you are using contains more than a single stream, and it was calibrated
    // then the SDK provides a way of getting the transformation between any two streams (if such exists)
    rs2_extrinsics extrinsics;
    int flag = 0;
    while(flag == 0){
        try
        {
            // Given two streams, use the get_extrinsics_to() function to get the transformation from the stream to the other stream
            extrinsics = from_stream.get_extrinsics_to(to_stream);
            std::cout << "Translation Vector : [" << extrinsics.translation[0] << "," << extrinsics.translation[1] << "," << extrinsics.translation[2] << "]\n";
            std::cout << "Rotation Matrix    : [" << extrinsics.rotation[0] << "," << extrinsics.rotation[3] << "," << extrinsics.rotation[6] << "]\n";
            std::cout << "                   : [" << extrinsics.rotation[1] << "," << extrinsics.rotation[4] << "," << extrinsics.rotation[7] << "]\n";
            std::cout << "                   : [" << extrinsics.rotation[2] << "," << extrinsics.rotation[5] << "," << extrinsics.rotation[8] << "]" << std::endl;
            flag = 1;
        }
        catch (const std::exception& e)
        {
            std::cerr << "Failed to get extrinsics for the given streams. " << e.what() << std::endl;
        }
    }
    return extrinsics;
}

Vec3d get_translation(rs2_extrinsics extrinsics){
    Vec3d T;
    T[0] = extrinsics.translation[0];
    T[1] = extrinsics.translation[1];
    T[2] = extrinsics.translation[2];
    return T;
}

Mat get_rotation(rs2_extrinsics extrinsics){
    Mat R = Mat(3, 3, CV_64F);
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            R.at<double>(i,j) = extrinsics.rotation[j*3+i];
        }
    }
    return R;
}

Mat get_R_T(Mat R, Vec3d T){
    Mat H = Mat::zeros(4, 4, CV_64F);
    int i, j;
    for (i = 0; i < 3; i++){
        for (j = 0; j < 3; j++){
            H.at<double>(i,j) = R.at<double>(i,j);
        }
    }
    for (i = 0; i < 3; i++){
        H.at<double>(i,3) = T[i];
    }
    H.at<double>(3,3) = 1.0;
    return H;
}

Mat get_tf(Mat R_0_1, Vec3d T_0_1, Mat R_1_2, Vec3d T_1_2){
    Mat H_0_1, H_1_2, H_0_2;
    H_0_1 = get_R_T(R_0_1, T_0_1);
    H_1_2 = get_R_T(R_1_2, T_1_2);
    H_0_2 = H_0_1 * H_1_2;
    return H_0_2;
}

/* Transform 3D coordinates relative to one sensor to 3D coordinates relative to another viewpoint */
cv::Point3d transform_point_to_point(Mat R, Vec3d T, rs2::vertex from_point)
{   
    double to_point[3];
    for (int i = 0; i < 3; i++){
        to_point[i] = R.at<double>(i,0) * from_point.x;
        to_point[i] += R.at<double>(i,1) * from_point.y;
        to_point[i] += R.at<double>(i,2) * from_point.z + T[i];
    }
    cv::Point3d outpoint(to_point[0], to_point[1], to_point[2]);
    return outpoint;
}


std::vector<cv::Point3d> transform_points(Mat R, Vec3d T, rs2::points from_points){
    std::vector<cv::Point3d> outpoints;
    cv::Point3d outpoint;
    size_t size = from_points.size();
    const rs2::vertex *from_vertices = from_points.get_vertices();
    for (size_t i = 0; i < size; i++){
        outpoint = transform_point_to_point(R, T, from_vertices[i]);
        outpoints.push_back(outpoint);
    }
    //std::cout<<"from: "<<from_vertices[20000].x<<", "<<from_vertices[20000].y<<", "<<from_vertices[20000].z << std::endl;
    //std::cout<<"to: "<< outpoints[20000].x<<", "<< outpoints[20000].y<<", "<< outpoints[20000].z<< std::endl;
    return outpoints;
}


/* Given a point in 3D space, compute the corresponding pixel coordinates in an image with no distortion or forward distortion coefficients produced by the same camera */
void project_point_to_pixel(double pixel[2], const struct rs2_intrinsics * intrin, const double point[3])
{
    //assert(intrin->model != RS2_DISTORTION_INVERSE_BROWN_CONRADY); // Cannot project to an inverse-distorted image

    double x = point[0] / point[2], y = point[1] / point[2];

    if(intrin->model == RS2_DISTORTION_MODIFIED_BROWN_CONRADY)
    {

        double r2  = x*x + y*y;
        double f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
        x *= f;
        y *= f;
        double dx = x + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
        double dy = y + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
        x = dx;
        y = dy;
    }
    if (intrin->model == RS2_DISTORTION_FTHETA)
    {
        double r = sqrt(x*x + y*y);
        double rd = (1.0f / intrin->coeffs[0] * atan(2 * r* tan(intrin->coeffs[0] / 2.0f)));
        x *= rd / r;
        y *= rd / r;
    }

    pixel[0] = x * intrin->fx + intrin->ppx;
    pixel[1] = y * intrin->fy + intrin->ppy;
}


/* Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients, compute the corresponding point in 3D space relative to the same camera */
void deproject_pixel_to_point(double point[3], const struct rs2_intrinsics * intrin, const double pixel[2], double depth)
{
    assert(intrin->model != RS2_DISTORTION_MODIFIED_BROWN_CONRADY); // Cannot deproject from a forward-distorted image
    assert(intrin->model != RS2_DISTORTION_FTHETA); // Cannot deproject to an ftheta image
    //assert(intrin->model != RS2_DISTORTION_BROWN_CONRADY); // Cannot deproject to an brown conrady model

    double x = (pixel[0] - intrin->ppx) / intrin->fx;
    double y = (pixel[1] - intrin->ppy) / intrin->fy;
    if(intrin->model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
    {
        double r2  = x*x + y*y;
        double f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
        double ux = x*f + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
        double uy = y*f + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
        x = ux;
        y = uy;
    }
    point[0] = depth * x;
    point[1] = depth * y;
    point[2] = depth;
}

void MyEllipse( Mat img, double angle ){
    int thickness = 15;
    int lineType = 8;
    int w = 500;
    ellipse( img,
        Point( w/2, w/2 ),
        Size( w/4, w/16 ),
        angle,
        0,
        360,
        Scalar( 255, 0, 0 ),
        thickness,
        lineType );
}