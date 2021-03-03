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


/* ---------------------------- Main Function ---------------------------------------*/
// ENTRY POINT
int main(int argc, char** argv )
{
	int ret;
	int fd;
	int i;
	struct v4l2_capability cap;
	long frame=0;     // First frame number enumeration
	char video[20];   // To store Video Port Device
	char label[50];   // To display the information
	char thermal_sensor_name[20];  // To store the sensor name
	char filename[60];  // PATH/File_count
	char folder_name[30];  // To store the folder name
        char video_frames_str[30];
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
	printf(WHT ">>> Buffer lenght=" YEL "%i" WHT "\n", bufferinfo.length);

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
	Size size(640,512);
	Mat thermal16_linear_zoom;   // (height,width, CV_8U, 1);    // Final representation
	Mat thermal_rgb_zoom;   // (height,width, CV_8U, 1);    // Final representation

	// Read frame, do AGC, paint frame
	for (;;) {

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


		// -----------------------------
		// RAW16 DATA
        AGC_Basic_Linear(thermal16, thermal16_linear, height, width);

        // Display thermal after 16-bits AGC... will display an image
        if (zoom_enable==0) {
            sprintf(label, "%s : RAW16  Linear", thermal_sensor_name);
            imshow(label, thermal16_linear);
        } else {
            resize(thermal16_linear, thermal16_linear_zoom, size);
            sprintf(label, "%s : RAW16  Linear Zoom", thermal_sensor_name);
            imshow(label, thermal16_linear_zoom);
        }

        if (record_enable==1) {
            sprintf(filename, "%s_raw16_%lu.tiff", thermal_sensor_name, frame);
            imwrite(filename, thermal16 , compression_params );
            sprintf(filename, "%s_agc_%lu.tiff", thermal_sensor_name, frame);
            imwrite(filename, thermal16_linear , compression_params );
            frame++;
        }
		
		// Press 'q' to exit
		if( waitKey(1) == 'q' ) { // 0x20 (SPACE) ; need a small delay !! we use this to also add an exit option
			printf(WHT ">>> " RED "'q'" WHT " key pressed. Quitting !\n");
			break;
		}
		// Stop if frame limit reached.
		if (video_frames>0 && frame+1 > video_frames) {
			printf(WHT ">>>" GRN "'Done'" WHT " Frame limit reached, Quitting !\n");
			break;
		}
	}
	// Finish Loop . Exiting.

	// Deactivate streaming
	if( ioctl(fd, VIDIOC_STREAMOFF, &type) < 0 ){
		perror(RED "VIDIOC_STREAMOFF" WHT);
		exit(1);
	};

	close(fd);
	return EXIT_SUCCESS;
}
