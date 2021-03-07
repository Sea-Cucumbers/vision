// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering
#include "util.hpp"             // Get extrinsics
#include <algorithm>            // std::min, std::max

#include <stdio.h>
#include <fcntl.h>               // open, O_RDWR
#include <opencv2/opencv.hpp>
#include <unistd.h>              // close
#include <sys/ioctl.h>           // ioctl
#include <asm/types.h>           // videodev2.h
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/videodev2.h>

using namespace cv;

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);


int main() {
    /*************** Some initialization stuff for RealSense **************/

    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Pointcloud Example");
    // Construct an object to manage view state
    glfw_state app_state;
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);

    rs2::pointcloud pc;
    rs2::points points_i;
	rs2::points points;
    rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_INFRARED, 1);
	cfg.enable_stream(RS2_STREAM_INFRARED, 2);
	cfg.enable_stream(RS2_STREAM_DEPTH);
	cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGBA8);
	rs2::pipeline_profile profile;
	rs2::colorizer color_map;
	
	std::vector<Point2d> pixels_c;
	std::vector<Point3d> points_c;


	Mat rsimage_zoom;
	Mat rsimage_zoom_gray;
	Mat rsimagec_gray;
	Mat rsimagec_zoom;
    int frame_num = 0;
    char filename_rs[60];

    Mat rsimagec_blurred;
    Mat rsimagec_rgb;
    Mat rsimagec_hls;
    Mat rsimagec_segmented;
    Mat mask1, mask2;
	int flag  = 0;

    SimpleBlobDetector::Params params;
    init_bolb_detector_params(params);

    // Start streaming with default recommended configuration
	while (flag == 0){
		try {
			profile = pipe.start();
			flag = 1;
		} catch (const rs2::error & e) {
			std::cout << "Could not set Vars, will try again." << std::endl;
			std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
		} catch (const std::exception & e) {
			std::cout << "Could not set Vars, will try again." << std::endl;
			std::cerr << e.what() << std::endl;
		}
	}
	std::cout << "Device found!" << std::endl;

    // Find camera parameters
    auto ir1_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::stream_profile>();
	auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::stream_profile>();
	rs2_extrinsics extrinsics = get_extrinsics(ir1_stream, color_stream);
	Mat R_c_i = get_rotation(extrinsics);
	Vec3d T_c_i = get_translation(extrinsics);
	std::cout << "R_c_i = " << R_c_i << std::endl;
	std::cout << "T_c_i = " << T_c_i << std::endl;

    // try retrieving rgb intrinsics
	auto intrinsics = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    Mat K, D;
    double kd[9] = {615.084, 0.0, 323.046, 0.0, 615.019, 242.301, 0.0,0.0,1.0};
    double dd[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    K = Mat(3,3,CV_64F,kd);
    D = Mat(1,5,CV_64F,dd);
	std::cout << "K = " << K << std::endl;
	std::cout << "D = " << D << std::endl;


    while (app) {
        // ----------------Wait for the next set of frames from RealSense
        auto frames = pipe.wait_for_frames();
        //auto depth = frames.get_depth_frame();

		rs2::frame depth_frame = frames.get_depth_frame();
		rs2::frame color_frame = frames.get_color_frame();
        // Query frame size (width and height)
        const int w = depth_frame.as<rs2::video_frame>().get_width();
        const int h = depth_frame.as<rs2::video_frame>().get_height();
		const int w1 = color_frame.as<rs2::video_frame>().get_width();
        const int h1 = color_frame.as<rs2::video_frame>().get_height();
		//std::cout << w1<<",  " << h1 << std::endl;
        // Create OpenCV matrix of size (w,h) from the colorized depth data
		Mat rsimagec(Size(w1, h1), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
        Mat rsimage(Size(w, h), CV_8UC3, (void*)depth_frame.apply_filter(color_map).get_data(), Mat::AUTO_STEP);
		//std::cout<<"good line281..."<<std::endl;
		//imshow("Realsense colorized depth img", rsimage);
	
        pc.map_to(color_frame);
        points_i = pc.calculate(depth_frame);

        // TODO Step 1: Find colored blobs in colored frame, draw
        //For segmenting the image in RGB format.
        cvtColor(rsimagec, rsimagec_rgb, COLOR_BGR2RGB);
        gammaCorrection(rsimagec_rgb, rsimagec_rgb, 0.45); // adjust brightness
        cvtColor(rsimagec_rgb, rsimagec_hls, COLOR_BGR2HLS);
        //GaussianBlur(rsimagec, rsimagec_blurred, Size(5,5), 0, 0);
        inRange(rsimagec_hls, cv::Scalar(0, 50, 35), cv::Scalar(50, 142, 255), mask1);
        inRange(rsimagec_hls, cv::Scalar(160, 50, 35), cv::Scalar(180, 142, 255), mask2);
        bitwise_or(mask1,  mask2, rsimagec_segmented); // blob is 255
        //threshold(rsimagec_segmented, rsimagec_segmented, 0, 255, 1); // binary inverted
        //GaussianBlur(rsimagec_segmented, rsimagec_segmented, Size(5,5), 0, 0);
        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        // Detect blobs.
        std::vector<KeyPoint> keypoints;
        detector->detect(rsimagec_segmented, keypoints);
        

        // Draw detected blobs as green circles.
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
        Mat im_with_keypoints, rsimagec_segmented_rgb;
        cvtColor(rsimagec_segmented, rsimagec_segmented_rgb, COLOR_GRAY2RGB);
        drawKeypoints(rsimagec_segmented_rgb, keypoints, im_with_keypoints, Scalar(0,255,0));
        //DrawMatchesFlags::DRAW_RICH_KEYPOINTS 
        
        // Show blobs
        //imshow("image", rsimagec_segmented);
        Mat buf2;
		Mat imgarray[] = {rsimagec_rgb, im_with_keypoints};	
        //Mat imgarray[] = {mask1, mask2};	
	    hconcat(imgarray, 2, buf2);
        imshow("image", buf2);

        // Step 2: Transform point could and project onto colored frame
        Mat R_i = Mat::eye(3,3,CV_64F);
		Vec3d T_i(0,0,0);
        // transform points from infra1 frame to color frame
		points_c = transform_points(R_c_i, T_c_i, points_i);

        projectPoints(points_c, R_i, T_i, K, D, pixels_c);

        // TODO Step 3: Find the 3D coordinate corresponding to 
        // the blob in 2D; average that, print
        std::cout << "num keypoints: " << keypoints.size() << std::endl;

        if (keypoints.size() == 0){
            std::cout << "bad!" << std::endl;
            continue;
        }
        KeyPoint best_keypoint;
        double min_z = 0;
        int best_point_idx = 0;
        for (int k = 0; k < keypoints.size(); k++){
            KeyPoint curr_keypoint = keypoints[k];
            Point2d pixel = curr_keypoint.pt;
            //std::cout << pixel.x << ", " << pixel.y << std::endl;
            double z = 0;
            int i;
            for (i = 0; i < pixels_c.size(); i++){

                if (pixels_c[i].x - pixel.x < 2 && pixels_c[i].y - pixel.y < 2){
                    z = points_c[i].z;
                    break;
                }
            }
            if (min_z == 0 || z < min_z){
                min_z = z;
                best_keypoint = curr_keypoint;
                best_point_idx = i;
            }
        }
        if (best_point_idx == 0){
            std::cout << "bad" << std::endl;
            continue;
        }
        Point3d best_point = points_c[best_point_idx];
        std::cout << best_point.x << ", " << best_point.y << ", " << best_point.z << std::endl;


        // display colored point cloud
        Mat rsimagec_bgr;
        cvtColor(rsimagec_rgb, rsimagec_bgr, COLOR_RGB2BGR);
        app_state.tex.upload0(rsimagec_bgr);
        draw_pointcloud(app.width(), app.height(), app_state, points_c, pixels_c, points_i);

        // Press 'c' to capture frames

		if( waitKey(1) == 'c' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
			printf("Capturing frames %d...\n", frame_num);
			sprintf(filename_rs, "../samples/red/red_%04d.png", frame_num);
            imwrite(filename_rs, rsimagec_rgb);
            frame_num++;
		}

		
        // Press 'q' to exit
		if( waitKey(1) == 'q' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
			printf("q key pressed. Quitting !\n");
			break;
		}
    }


	return 0;

}
