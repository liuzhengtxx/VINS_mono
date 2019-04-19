#pragma once
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

extern int ROW;
extern int COL;
extern int FOCAL_LENGTH;
const int NUM_OF_CAM = 1;


extern std::string IMAGE_TOPIC;// "/cam0/image_raw"
extern std::string IMU_TOPIC;//"/imu0"
extern std::string FISHEYE_MASK;
extern std::vector<std::string> CAM_NAMES;//camera
extern int MAX_CNT;//150  max feature number in feature tracking
extern int MIN_DIST;// 30 min distance between two features 
extern int WINDOW_SIZE;
extern int FREQ;//freq: 10 frequence (Hz) of publish tracking result. 
                //At least 10Hz for good estimation. If set 0, the frequence will be same as raw image 
extern double F_THRESHOLD;//1.0 ransac threshold (pixel)
extern int SHOW_TRACK;//1   publish tracking image as topic
extern int STEREO_TRACK;
extern int EQUALIZE;//1  if image is too dark or light, trun on equalize to find enough features
extern int FISHEYE;//0 if using fisheye, trun on it. A circle mask will be loaded to remove edge noisy points
extern bool PUB_THIS_FRAME;

void readParameters(ros::NodeHandle &n);
