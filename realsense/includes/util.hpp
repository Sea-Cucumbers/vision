#include <iostream>
#include <iomanip>
#include <map>
#include <utility>
#include <vector>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <unordered_map>
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


std::string get_string_2(double x_, double y_){
    int x = (int)round(x_);
    int y = (int)round(y_);
    std::string res = std::to_string(x) + ", "  + std::to_string(y);
    return res;
}

void projectPoints0(std::vector<cv::Point3d> &points, Mat &R, Vec3d &T, Mat &K, Mat &D, std::vector<cv::Point2d> &pixels, std::unordered_map<std::string, cv::Point3d> &u){
    double fx, fy, cx, cy;
    fx = K.at<double>(0,0);
    fy = K.at<double>(1,1);
    cx = K.at<double>(0,2);
    cy = K.at<double>(1,2);
    for (int i = 0; i < points.size(); i++){
        cv::Point3d point = points.at(i);
        double pixel[2];
        pixel[0] = fx * point.x / z + cx;
        pixel[1] = fy * point.y / z + cy;
        pixels.at(i).x = pixel[0];
        pixels.at(i).y = pixel[1];
        std::string pixel_str = get_string_2(pixel[0], pixel[1]);
        u[pixel_str] = point;
    }
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

void init_bolb_detector_params(SimpleBlobDetector::Params &params){
    params.blobColor = 255;
    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 100;
    params.maxArea = 4000;
    // Filter by Circularity
    params.filterByCircularity = false;
    params.minCircularity = 0.2;
    // Filter by Convexity
    params.filterByConvexity = true;
    params.minConvexity = 0.9;
    // Filter by Inertia
    params.filterByInertia = true;
    params.minInertiaRatio = 0.5;
}

void gammaCorrection(const Mat &img, Mat &img_corrected, const double gamma_){
    CV_Assert(gamma_ >= 0);
    //! [changing-contrast-brightness-gamma-correction]
    Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for( int i = 0; i < 256; ++i)
        p[i] = saturate_cast<uchar>(pow(i / 255.0, gamma_) * 255.0);

    img_corrected = img.clone();
    LUT(img, lookUpTable, img_corrected);
    //! [changing-contrast-brightness-gamma-correction]
}

/*
void automatic_brightness_and_contrast(image, clip_hist_percent=1):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Calculate grayscale histogram
    hist = cv2.calcHist([gray],[0],None,[256],[0,256])
    hist_size = len(hist)

    # Calculate cumulative distribution from the histogram
    accumulator = []
    accumulator.append(float(hist[0]))
    for index in range(1, hist_size):
        accumulator.append(accumulator[index -1] + float(hist[index]))

    # Locate points to clip
    maximum = accumulator[-1]
    clip_hist_percent *= (maximum/100.0)
    clip_hist_percent /= 2.0

    # Locate left cut
    minimum_gray = 0
    while accumulator[minimum_gray] < clip_hist_percent:
        minimum_gray += 1

    # Locate right cut
    maximum_gray = hist_size -1
    while accumulator[maximum_gray] >= (maximum - clip_hist_percent):
        maximum_gray -= 1

    # Calculate alpha and beta values
    alpha = 255 / (maximum_gray - minimum_gray)
    beta = -minimum_gray * alpha

    '''
    # Calculate new histogram with desired range and show histogram 
    new_hist = cv2.calcHist([gray],[0],None,[256],[minimum_gray,maximum_gray])
    plt.plot(hist)
    plt.plot(new_hist)
    plt.xlim([0,256])
    plt.show()
    '''

    auto_result = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    return (auto_result, alpha, beta)
*/