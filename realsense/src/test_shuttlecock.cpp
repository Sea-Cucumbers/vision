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
	int flag  = 0;
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

        // Step 2: Transform point could and project onto colored frame
        Mat R_i = Mat::eye(3,3,CV_64F);
		Vec3d T_i(0,0,0);
        // transform points from infra1 frame to color frame
		points_c = transform_points(R_c_i, T_c_i, points_i);

        projectPoints(points_c, R_i, T_i, K, D, pixels_c);

        // TODO Step 3: Find the 3D coordinate corresponding to 
        // the blob in 2D; average that, print


        // display colored point cloud
        app_state.tex.upload0(rsimagec);
        draw_pointcloud(app.width(), app.height(), app_state, points_c, pixels_c, points_i);

        // Press 'q' to exit
		if( waitKey(1) == 'q' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
			printf("q key pressed. Quitting !\n");
			break;
		}
    }


	return 0;

}
