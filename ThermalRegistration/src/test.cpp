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

void test_get_R_T(){
    float data[9] = {0,1,2,3,4,5,6,7,8};
    Mat R = Mat(3,3,CV_64F, data);
    Vec3d T(10, 11, 12);
    Mat out = get_R_T(R, T);
    std::cout << out << std::endl;
}

void test_mul(){
    float data1[4] = {0,1,2,3};
    Mat R1 = Mat(2,2,CV_64F, data1);
    float data2[4] = {2,1,0,1};
    Mat R2 = Mat(2,2,CV_64F, data2);
    Mat R = R1 * R2;
    std::cout << R << std::endl;
}



int main(){
    Point3d p(0.474524, -1.36015, 2.628);
    std::cout<<"from: "<<p.x<<", "<<p.y<<", "<<p.z << std::endl;

    return 0;
}