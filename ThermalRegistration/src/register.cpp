/*
 * Use a point cloud, camera extrinsics, 
 * and 2D thermal image to perform thermal registration
 *
 */ 
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Short list of convenience functions for rendering
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

#include "boson_image.h"
using namespace cv;

int main() { 

    // Step 0: Initialize RealSense and Boson

    //Some initialization stuff for Boson 
    int ret;
	int fd;
	int i;
	struct v4l2_capability cap;
	int frame_num=42;     // First frame number enumeration
	char video[20];   // To store Video Port Device
	char label[50];   // To display the information
	char thermal_sensor_name[20];  // To store the sensor name
	char filename_rs[60];  // PATH/File_count
	char filename_boson[60];
	char folder_name[30];  // To store the folder name
    char video_frames_str[30];
	char stereo_calib_filename[80];
	char boson_calib_filename[80];
	// Default Program options
	int  video_mode=RAW16;
	int  video_frames=0;
	int  zoom_enable=1; // ZOOM to 640x512 is enabled
	int  record_enable=0;
	sensor_types my_thermal=Boson320;

	// To record images
	std::vector<int> compression_params;
	compression_params.push_back(IMWRITE_PXM_BINARY);

	// Video device by default
	int camera_port = 2;
    sprintf(video, "/dev/video%d", camera_port);
	sprintf(thermal_sensor_name, "Boson_320");

    // We open the Video Device
	printf(WHT ">>> " YEL "%s" WHT " selected\n", video);
	if((fd = open(video, O_RDWR)) < 0){
		perror(RED "Error : OPEN. Invalid Video Device" WHT "\n");
		exit(1);
	}

	// Check VideoCapture mode is available
	if(ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0){
	    perror(RED "ERROR : VIDIOC_QUERYCAP. Video Capture is not available" WHT "\n");
	    exit(1);
	}

	if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)){
		fprintf(stderr, RED "The device does not handle single-planar video capture." WHT "\n");
		exit(1);
	}

    struct v4l2_format format;
	
	CLEAR(format);

    // Requiring thermal 16 bits mode
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_Y16;

	// Common varibles
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width = width;
	format.fmt.pix.height = height;

	// request desired FORMAT
	if(ioctl(fd, VIDIOC_S_FMT, &format) < 0){
		perror(RED "VIDIOC_S_FMT" WHT);
		exit(1);
	}

	// we need to inform the device about buffers to use.
	// and we need to allocate them.
	// we’ll use a single buffer, and map our memory using mmap.
	// All this information is sent using the VIDIOC_REQBUFS call and a
	// v4l2_requestbuffers structure:
	struct v4l2_requestbuffers bufrequest;
	bufrequest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	bufrequest.memory = V4L2_MEMORY_MMAP;
	bufrequest.count = 1;   // we are asking for one buffer

	if(ioctl(fd, VIDIOC_REQBUFS, &bufrequest) < 0){
		perror(RED "VIDIOC_REQBUFS" WHT);
		exit(1);
	}

	// Now that the device knows how to provide its data,
	// we need to ask it about the amount of memory it needs,
	// and allocate it. This information is retrieved using the VIDIOC_QUERYBUF call,
	// and its v4l2_buffer structure.

	struct v4l2_buffer bufferinfo;
	memset(&bufferinfo, 0, sizeof(bufferinfo));

	bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	bufferinfo.memory = V4L2_MEMORY_MMAP;
	bufferinfo.index = 0;

	if(ioctl(fd, VIDIOC_QUERYBUF, &bufferinfo) < 0){
		perror(RED "VIDIOC_QUERYBUF" WHT);
		exit(1);
	}


	// map fd+offset into a process location (kernel will decide due to our NULL). lenght and
	// properties are also passed
	printf(WHT ">>> Image width  =" YEL "%i" WHT "\n", width);
	printf(WHT ">>> Image height =" YEL "%i" WHT "\n", height);
	printf(WHT ">>> Buffer length=" YEL "%i" WHT "\n", bufferinfo.length);

	void * buffer_start = mmap(NULL, bufferinfo.length, PROT_READ | PROT_WRITE,MAP_SHARED, fd, bufferinfo.m.offset);

	if(buffer_start == MAP_FAILED){
		perror(RED "mmap" WHT);
		exit(1);
	}

	// Fill this buffer with ceros. Initialization. Optional but nice to do
	memset(buffer_start, 0, bufferinfo.length);

	// Activate streaming
	int type = bufferinfo.type;
	if(ioctl(fd, VIDIOC_STREAMON, &type) < 0){
		perror(RED "VIDIOC_STREAMON" WHT);
		exit(1);
	}

	// Declarations for RAW16 representation
        // Will be used in case we are reading RAW16 format
	// Boson320 , Boson 640
	Mat thermal16(height, width, CV_16U, buffer_start);   // OpenCV input buffer  : Asking for all info: two bytes per pixel (RAW16)  RAW16 mode`
	Mat thermal16_linear(height,width, CV_8U, 1);         // OpenCV output buffer : Data used to display the video

	// Declarations for Zoom representation
    	// Will be used or not depending on program arguments
	Size size(640,480);
	Mat thermal16_linear_zoom;   // (height,width, CV_8U, 1);    // Final representation
	Mat thermal16_linear_zoom_undist;
	Mat thermal_rgb_zoom;   // (height,width, CV_8U, 1);    // Final representation
	Mat thermal_out;
	Mat lut0 = imread("../map.png");
	Mat lut;
	cvtColor(lut0, lut, COLOR_RGB2BGR);
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
	
	std::vector<Point2d> pixels_t;
	std::vector<Point3d> points_t;

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

	// Step 3: Back project pointcloud to Boson image
	// 3.1 - get the extrinsics between infra1 and colorcam: stored in rs
	//       and between colorcam and boson: load calib_stereo_rgbt.yml
	std::cout << "Reading extrinsics..." << std::endl;
	Mat R_t_c;
	double rd[9] = {0.9965285186230173, 0.001563261489606431, 0.08323741817553992, 0.004069965186811401, 0.9977134125406449, -0.06746392976901774, -0.08315255230233606, 0.06756850338742104, 0.9942435065896037};
	double td[3] = {-0.029657158210394333, 0.023256621947827637, -0.02349648777943659};

	R_t_c = Mat(3,3,CV_64F, rd);
	Vec3d T_t_c(td[0], td[1], td[2]);
	/*
	sprintf(stereo_calib_filename, "../../RGBDT_calib/calib_stereo_rgbt_cv.xml");
	FileStorage fsl1(stereo_calib_filename, FileStorage::READ);
	fsl1["R"] >> R_t_c;
	fsl1["T"] >> T_t_c;
	*/
	std::cout << "R_t_c = " << R_t_c << std::endl;
	std::cout << "T_t_c = " << T_t_c << std::endl;
	

	auto ir1_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::stream_profile>();
	auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::stream_profile>();
	rs2_extrinsics extrinsics = get_extrinsics(ir1_stream, color_stream);
	Mat R_c_i = get_rotation(extrinsics);
	Vec3d T_c_i = get_translation(extrinsics);
	std::cout << "R_c_i = " << R_c_i << std::endl;
	std::cout << "T_c_i = " << T_c_i << std::endl;

	// try retrieving rgb intrinsics
	auto intrinsics = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
	std::cout << "K of rgb: " << std::endl;
	std::cout << intrinsics.fx << "\t" << 0 << "\t" << intrinsics.ppx<<"\n"
	          << 0  << "\t" << intrinsics.fy <<"\t"<< intrinsics.ppy<<"\n"
			  << "0\t0\t1" << std::endl;
	for(auto value : intrinsics.coeffs)
    	std::cout<<value<<", ";
    std::cout<<std::endl;
    std::cout<<"distortion model: "<<intrinsics.model<<std::endl;
	/*
	auto ir2_stream = profile.get_stream(RS2_STREAM_INFRARED, 2);
	rs2_extrinsics e = ir1_stream.get_extrinsics_to(ir2_stream);
	Mat R_i2_i1 = get_rotation(e);
	Vec3d T_i2_i1 = get_translation(e);
	std::cout << "R_i2_i1 = " << R_i2_i1 << std::endl;
	std::cout << "T_i2_i1 = " << T_i2_i1 << std::endl;
	//auto baseline = e.translation[0];
	*/

	// 3.2 - get intrinsics of boson: load calib_thermal.yml
	std::cout << "Reading intrinsics..." << std::endl;
	Mat K, D;
	double kd[9] = {378.61542685834496, 0.0, 286.2484989291891, 0.0, 354.3170644790293, 268.007589783555, 0.0, 0.0, 1.0};
	double dd[5] = {-0.3136115547594644, -0.03417021290990186, 0.0019021090367376222, 0.006430856813556693, 0.46861168056753133};
	//double kd[9] = {3.8731199613695077e+02, 0., 2.9107593452496030e+02, 0., 3.8677461674607787e+02, 2.8774295490311647e+02, 0., 0., 1.};
	//double dd[5] = {-2.6850469535756538e-01, -2.5261398473044139e-01, 2.4643018209376264e-03, 1.5695775458624288e-03, 3.6144743392507711e-01};
	K = Mat(3,3,CV_64F, kd);
	D = Mat(1,5,CV_64F, dd);
	/*
	sprintf(boson_calib_filename, "../../RGBDT_calib/calib_thermal_cv.xml");
	FileStorage fsl2(boson_calib_filename, FileStorage::READ);
	fsl2["K"] >> K;
	fsl2["D"] >> D;
	*/
	std::cout << "K = " << K << std::endl;
	std::cout << "D = " << D << std::endl;
	int wid, hei, nrChannels;
    //unsigned char *data = stbi_load("../img7.png", &wid, &hei, &nrChannels, 0);
	// transformation from thermal frame to infra1 frame
	std::cout << "Calculating Extrinsics..." << std::endl;
	Mat H_t_i = get_tf(R_t_c, T_t_c, R_c_i, T_c_i);
	Mat R_t_i = H_t_i(Range(0,3), Range(0,3)).clone();
	Vec3d T_t_i(H_t_i.at<double>(0,3), H_t_i.at<double>(1,3), H_t_i.at<double>(2,3));
	std::cout << "R_t_i = "<<R_t_i << std::endl;
	std::cout << "T_t_i = "<<T_t_i << std::endl;
	std::cout << "Start streaming..." << std::endl;
    while (app) {
		// Step 1&2: Get point cloud from realsense and Boson (and save?)

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
        //points.export_to_ply("your_filename.ply", color);
		//std::cout<<"good line287..."<<std::endl;
        // ---------------- Wait for next frame from Boson
		// Put the buffer in the incoming queue.
		if(ioctl(fd, VIDIOC_QBUF, &bufferinfo) < 0){
			perror(RED "VIDIOC_QBUF" WHT);
			exit(1);
		}

		// The buffer's waiting in the outgoing queue.
		if(ioctl(fd, VIDIOC_DQBUF, &bufferinfo) < 0) {
			perror(RED "VIDIOC_QBUF" WHT);
			exit(1);
		}

		// RAW16 DATA
        AGC_Basic_Linear(thermal16, thermal16_linear, height, width);
		//std::cout<<"good line303..."<<std::endl;
		
        // Display thermal after 16-bits AGC... will display an image
        if (zoom_enable==0) {
            sprintf(label, "%s : RAW16  Linear", thermal_sensor_name);
            imshow(label, thermal16_linear);
        } else {
			
			//std::cout <<"here"<<std::endl;
            resize(thermal16_linear, thermal16_linear_zoom, size);

			//resize(rsimagec, rsimagec_zoom, size);
			//cvtColor(rsimagec_zoom, rsimagec_gray, COLOR_RGB2GRAY);
			//Mat buf2;
			//Mat imgarray[] = {thermal16_linear_zoom, rsimagec_gray};	
			//hconcat(imgarray, 2, buf2);
			//imshow("2 images", buf2);		
        }
        if (record_enable==1) {
            //sprintf(filename, "%s_raw16_%lu.tiff", thermal_sensor_name, frame);
            //imwrite(filename, thermal16 , compression_params );
			// Save boson image
            sprintf(filename_boson, "%04d_thermal.tiff", frame_num);
            imwrite(filename_boson, thermal16_linear_zoom , compression_params);
			// Save rs image
			sprintf(filename_rs, "%04d_rgb.png", frame_num);
			imwrite(filename_rs, rsimagec_gray);
            frame_num++;
        }
		
		//std::cout<<"good line335..."<<std::endl;
		// 3.3 - texture mapping 
		// transform points from infra1 frame to thermal frame 
		Mat R_i = Mat::eye(3,3,CV_64F);
		Vec3d T_i(0,0,0);
		points_t = transform_points(R_t_i, T_t_i, points_i);
		//std::cout<<points_i<<std::endl;
		// project points in thermal frame to boson image
		
		projectPoints(points_t, R_i, T_i, K, D, pixels_t);
		//std::cout<<"good line342..."<<std::endl;
		undistort(thermal16_linear_zoom, thermal16_linear_zoom_undist, K, D);
		//std::cout<<"good line344..."<<std::endl;
		// TODO write a match function
		//points = match(points_t, pixels_t);
		
		Mat thermal16_linear_zoom_undist_rgb;
    	// TODO Step 4: Visualize colored point cloud
        // Upload the color frame to OpenGL
		cvtColor(thermal16_linear_zoom_undist, thermal16_linear_zoom_undist_rgb, COLOR_GRAY2RGB);
		applyColorMap(thermal16_linear_zoom_undist_rgb, thermal_out, lut);
		//MyEllipse(thermal16_linear_zoom_undist_rgb, 0);
		//Mat img0 = imread("../img8.png");
		//Mat im0;
		//cvtColor(img0, im0, COLOR_BGR2RGB);
		//std::cout<< img<<std::endl;
		//Mat img0;
		//resize(img, img0, size);
		//Mat img0 = Mat::zeros(640,512,CV_8UC3);
		//img0.setTo(cv::Scalar(255,0,0));
		// load and generate the texture
		app_state.tex.upload0(thermal_out);
        //app_state.tex.upload1(data, wid, hei, nrChannels);

        // Draw the pointcloud
        draw_pointcloud(app.width(), app.height(), app_state, points_t, pixels_t, points_i);
		
		// Press 'c' to capture frames
		/*
		if( waitKey(1) == 'c' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
			printf("Capturing frames %d...\n", frame_num);
			sprintf(filename_boson, "../calib_samples/thermal/%04d_thermal.png", frame_num);
            imwrite(filename_boson, thermal16_linear_zoom );
			// Save rs image
			sprintf(filename_rs, "../calib_samples/rgb/%04d_rgb.png", frame_num);
			imwrite(filename_rs, rsimagec_gray);
            frame_num++;
		}
		*/
		
		// Press 'q' to exit
		if( waitKey(1) == 'q' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
			printf(WHT ">>> " RED "'q'" WHT " key pressed. Quitting !\n");
			break;
		}
    }

	// Deactivate streaming
	if( ioctl(fd, VIDIOC_STREAMOFF, &type) < 0 ){
		perror(RED "VIDIOC_STREAMOFF" WHT);
		exit(1);
	};

	close(fd);

	return 0;
}