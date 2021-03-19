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

#include <math.h>

using namespace cv;

// Helper functions
void register_glfw_callbacks(window& app, glfw_state& app_state);


int main(int argc, char *argv[]) {
    /*************** Some initialization stuff for RealSense **************/
/*
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Pointcloud Example");
    // Construct an object to manage view state
    glfw_state app_state;
    // register callbacks to allow manipulation of the pointcloud
    register_glfw_callbacks(app, app_state);
*/
    // process cmd line args
    if (argc != 2){
        std::cout << "Error: Bad argument count" << std::endl;
        return -1;
    }
    char *device_type_str = argv[1];
    int device_type;
    if (strcmp(device_type_str, "V1") == 0){
        device_type = DEVICE_GATE;
    }
    else if (strcmp(device_type_str, "V2")==0)
    {
        device_type = DEVICE_LARGE;
    }
    else if (strcmp(device_type_str, "V3")==0)
    {
        device_type = DEVICE_SHUTTLECOCK;
    }
    else if (strcmp(device_type_str, "B")==0)
    {
        device_type = DEVICE_BREAKER;
    }
    else{
        std::cout << "Error: No such device" << std::endl;
        return -1;
    }

    rs2::pointcloud pc;
    rs2::pipeline pipe;
	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_INFRARED, 1);
	cfg.enable_stream(RS2_STREAM_INFRARED, 2);
	cfg.enable_stream(RS2_STREAM_DEPTH);
	cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGBA8);
	rs2::pipeline_profile profile;
	rs2::colorizer color_map;
	
	std::vector<Point3d> points_c;

    // using gamma corrected images
    //Scalar blue_min(92, 63, 220);
    //Scalar blue_max(105, 128,255);

    // blue, original, circular
    Scalar blue_min(95, 21, 191);
    Scalar blue_max(110, 120,255);

    // blue, original, shuttlecock
    Scalar blue_min_2(98, 25, 160);
    Scalar blue_max_2(110, 178,255);

    // lime, original, spigot
    Scalar lime_min(40, 64, 25);
    Scalar lime_max(80, 204, 77);

    // the ball valves , corrected
    //Scalar blue2_min(92, 102, 77);
    //Scalar blue2_max(105, 204, 255);

    //Scalar blue3_min(92, 102, 63);
    //Scalar blue3_max(100, 128, 255);


    // orange, corrected
    //Scalar orange_min(9,102,20);
    //Scalar orange_max(15, 210, 255);

    // orange, original
    Scalar orange_min(5,77,102);
    Scalar orange_max(15, 204, 255);

    Scalar thresh_min, thresh_max;
    if (device_type == DEVICE_BREAKER){
        thresh_min = orange_min;
        thresh_max = orange_max;
    }
    else if (device_type == DEVICE_SHUTTLECOCK){
        thresh_min = blue_min_2;
        thresh_max = blue_max_2;
    }
    else{
        thresh_min = blue_min;
        thresh_max = blue_max;
    }



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


    while (1) {
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
        rs2::points points_i;
        pc.map_to(color_frame);
        points_i = pc.calculate(depth_frame);

        // Step 1: Find colored blobs in colored frame, draw
        //For segmenting the image in RGB format.
        cvtColor(rsimagec, rsimagec_rgb, COLOR_BGR2RGB);
        //gammaCorrection(rsimagec_rgb, rsimagec_rgb, 0.45); // adjust brightness
        cvtColor(rsimagec_rgb, rsimagec_hls, COLOR_BGR2HLS);

        // actual valves and breakers
        inRange(rsimagec_hls, thresh_min, thresh_max, rsimagec_segmented);
        if (device_type == DEVICE_GATE){
            inRange(rsimagec_hls, lime_min, lime_max, mask2);
            bitwise_or(rsimagec_segmented, mask2, rsimagec_segmented);
        }

        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        // Detect blobs.
        std::vector<KeyPoint> keypoints;
        detector->detect(rsimagec_segmented, keypoints);
        

        // Draw detected blobs as green circles.
        // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
        Mat im_with_keypoints, rsimagec_segmented_rgb;
        cvtColor(rsimagec_segmented, rsimagec_segmented_rgb, COLOR_GRAY2RGB);
        drawKeypoints(rsimagec_segmented_rgb, keypoints, im_with_keypoints, Scalar(0,255,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        


        // Step 2: Transform point could and project onto colored frame
        Mat R_i = Mat::eye(3,3,CV_64F);
		Vec3d T_i(0,0,0);
        // transform points from infra1 frame to color frame
        rs2::points points;
		points_c = transform_points(R_c_i, T_c_i, points_i);
        std::unordered_map<std::string, cv::Point3d> u;
        std::vector<Point2d> pixels_c;
        projectPoints0(points_c, R_i, T_i, K, D, pixels_c, u);
        std::cout << points_c.size() << ", " << pixels_c.size() << std::endl;
        // TODO Step 3: Find the 3D coordinate corresponding to 
        // the blob in 2D; average that, print
        std::cout << "num keypoints: " << keypoints.size() << std::endl;

        if (keypoints.size() == 0){
            std::cout << "no keypoint" << std::endl;
            continue;
        }

        // we compute and store the L2 distance and z value of each keypoint
        KeyPoint best_keypoint;
        Point3d best_point;
        Point2d best_pixel;
        double min_z = 0;
        int best_point_idx = 0;
        std::priority_queue<Blob> blob_pq;
        for (int k = 0; k < keypoints.size(); k++){
            KeyPoint curr_keypoint = keypoints.at(k);
            Point2d pixel = curr_keypoint.pt;
            int pixel_x = (int)pixel.x;
            int pixel_y = (int)pixel.y;
            size_t area = curr_keypoint.size;
            int r = (int)sqrt(area);
            double z = 0;
            cv::Point3d best_point_tmp;
            cv::Point2d best_pixel_tmp;
            double best_dist_tmp;
            // check every pixel in the blob, find 3D coords
            // and get one 3D coord representing the blob
            for (int i = -r; i < r+1; i++){
                int x = pixel_x + i;
                for (int j = -r; j < r+1; j++){
                    int y = pixel_y + j;
                    std::string key = get_string_2(x, y);
                    auto got = u.find(key);
                    if (got != u.end()){
                        Point3d pgot = got->second;
                        if (z == 0 || z > pgot.z){
                            z = pgot.z;
                            best_point_tmp = pgot;
                            Point2d px((double)x, (double)y);
                            best_pixel_tmp = px;
                            best_dist_tmp = sqrt(pgot.x * pgot.x + pgot.y * pgot.y + z * z);
                        }
                    }

                }
            }
            // push the blob to pq
            Blob blob(best_dist_tmp, best_point_tmp, best_pixel_tmp, curr_keypoint);
            blob_pq.push(blob);
            // check if this blob is the closest
            /*
            if (min_z == 0 || z < min_z){
                min_z = z;
                best_point = best_point_tmp;
                best_pixel = best_pixel_tmp;
            }
            */
        }
        std::vector<Blob> breakers;
        Blob best_blob;
        // for valves, we find one best keypoint
        if (device_type != DEVICE_BREAKER){
            best_blob = blob_pq.top();
            blob_pq.pop();
            std::cout << "Pixel: " << best_blob.pixel.x << ", " << best_blob.pixel.y << "\tPoint: " << best_blob.point.x << ", " << best_blob.point.y << ", " << best_blob.point.z << std::endl;
        }
        // for breakers, we find 3 best keypoints from PQ
        else {
            for (int i = 0; i < 3; i++){
                best_blob = blob_pq.top();
                breakers.push_back(best_blob);
                blob_pq.pop();
                std::cout << "Breaker " << i << ": " <<std::endl;
                std::cout << "Pixel: " << best_blob.pixel.x << ", " << best_blob.pixel.y << "\tPoint: " << best_blob.point.x << ", " << best_blob.point.y << ", " << best_blob.point.z << std::endl;
            }
            std::cout << std::endl;
        }



        //// Determine valve configuration (vertically/horizontally mounted)
        Mat rethresh_mask = Mat::zeros(Size(w1, h1), CV_8UC1);
        Mat rsimage_rethreshed;
        int r;
        int device_config;
        int device_state;
        // if it's a valve:
        if (device_type != DEVICE_BREAKER){
            r = 100;
            // if it's a shuttlecock valve, we just check its #pixels
            if (device_type == DEVICE_SHUTTLECOCK){
                std::cout << best_blob.keypoint.size << std::endl;
                if (best_blob.keypoint.size < 45){
                    device_config = DEVICE_VERTICAL;
                }
                else{
                    device_config = DEVICE_HORIZONTAL;
                }
            }
            else if (device_type == DEVICE_GATE){
                r = 90;
            }
            else if (device_type == DEVICE_LARGE){
                r = 170;
            }
            // re-threshold the image. we only keep the valve of interest
            int xc, yc;
            xc = best_blob.pixel.x;
            yc = best_blob.pixel.y;
            cv::rectangle(rethresh_mask, Point(xc-r, yc-r), Point(xc+r, yc+r), 255, FILLED);
            bitwise_and(rethresh_mask, rsimagec_segmented, rsimage_rethreshed);

            // find its contours and fit an ellipse
            std::vector<std::vector<cv::Point>> contours;
            findContours(rsimage_rethreshed, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
            if (contours.size() == 0) {
                std::cout << "No contour!" << std::endl;
                continue;
            }
            double best_area = cv::contourArea(contours[0]);
            size_t best_idx = 0;
            for (size_t i = 1; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);
                if (area > best_area) {
                best_idx = i;
                best_area = area;
                }
            }
            if (contours[best_idx].size() < 5) {
                continue;
            }
            cv::RotatedRect ell = fitEllipse(contours[best_idx]);
            cv::ellipse(rsimagec_rgb, ell, cv::Scalar(0, 255, 0), 5);

            // check the bounding box's shape
            float wid = ell.size.width;
            float hei = ell.size.height;
            float ratio = get_rect_ratio(wid, hei);
            std::cout << "Ratio: " << ratio << std::endl;
            if (device_type != DEVICE_SHUTTLECOCK){
                if (ratio < 0.4){
                    device_config = DEVICE_VERTICAL;
                }
                else {
                    device_config = DEVICE_HORIZONTAL;
                }
            }
            else{ // shuttlecock, we also determine device state
                if (device_config == DEVICE_HORIZONTAL){
                    // check bounding rec
                    Rect br = ell.boundingRect();
                    if (br.width > br.height)
                        device_state = VALVE_CLOSED;
                    else
                        device_state = VALVE_OPEN;

                }
                else{ // we count the #pixels
                    if (best_blob.keypoint.size > 30)
                        device_state = VALVE_OPEN;
                    else
                        device_state = VALVE_CLOSED;
                }
            }
            std::cout << get_valve_string(device_config, device_state, device_type) << std::endl;
        }
        else{
            int breaker_state[3];
            //// Determine breaker state (up/down)
            // sort based on pixel x
            Blob breaker[3];
            double x_1, x_2, x_3;
            double min_x = -1;
            int b1_idx = 0;
            double max_x = -1;
            int b3_idx = 0;
            int b2_idx = 0;
            bool isb1b3[3] = {false, false, false};
            for (int i = 0; i < 3; i++){
                double cur_x = breakers.at(i).pixel.x;
                if (min_x == -1 || cur_x < min_x){
                    min_x = cur_x;
                    b1_idx = i;
                }
                if (max_x == -1 || cur_x > max_x){
                    max_x = cur_x;
                    b3_idx = i;
                }
            }
            isb1b3[b1_idx] = true;
            isb1b3[b3_idx] = true;
            breaker[0] = breakers.at(b1_idx);
            breaker[2] = breakers.at(b3_idx);
            for (int i = 0; i < 3; i++){
                if(isb1b3[i] == false)
                    b2_idx = i;
            }
            breaker[1] = breakers.at(b2_idx);
            // draw text on the breakders 
            for (int i = 0; i < 3; i++){
                std::string tx = "B" + std::to_string(i+1);
                putText(rsimagec_rgb, tx, breaker[i].pixel, FONT_HERSHEY_SIMPLEX, 2,Scalar(0,255,0), 3);
                if (breaker[i].point.y < -0.05)
                    breaker_state[i] = BREAKER_UP;
                else
                    breaker_state[i] = BREAKER_DOWN;
            }
            std::cout << get_breaker_string(breaker_state[0]) << ", " << get_breaker_string(breaker_state[1]) << ", " << get_breaker_string(breaker_state[2]) << std::endl;
        }


        // Show blobs
        Mat buf2;
        //Mat rsimagec_rgb_backup;
        //cvtColor(rsimagec, rsimagec_rgb_backup, COLOR_BGR2RGB);
		Mat imgarray[] = {rsimagec_rgb, im_with_keypoints};	
	    hconcat(imgarray, 2, buf2);
        imshow("image", buf2);


        // Press 'c' to capture frames
		if( waitKey(1) == 'c' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
			printf("Capturing frames %d...\n", frame_num);
			sprintf(filename_rs, "../samples/red/new_%04d.png", frame_num);
            imwrite(filename_rs, buf2);
            frame_num++;
		}

/*
        // display colored point cloud
        Mat rsimagec_bgr;
        cvtColor(rsimagec_rgb, rsimagec_bgr, COLOR_RGB2BGR);
        app_state.tex.upload0(rsimagec_bgr);
        draw_pointcloud(app.width(), app.height(), app_state, points_c, pixels_c, points_i);
*/

		
        // Press 'q' to exit
		if( waitKey(1) == 'q' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
			printf("q key pressed. Quitting !\n");
			break;
		}
    }


	return 0;

}
