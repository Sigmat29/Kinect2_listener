#ifndef HEADERS_H1
#define HEADERS_H1


#include <ros/ros.h>
#include <ros/package.h>

#include <tf/transform_listener.h>
#include <termios.h>  //dla getch()

#include <iostream>
#include <image_transport/image_transport.h>

#include "cv_bridge/cv_bridge.h"

#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "std_msgs/Empty.h"
#include <stdio.h>
#include <string>
#include <std_srvs/Empty.h>
#include "std_msgs/String.h"
#include <sensor_msgs/image_encodings.h>

#include <vector>


#include <dynamic_reconfigure/server.h>

#include <kinect2_tracker/set_kinect_v2Config.h>
#include <kinect2_tracker/dataHSV.h>
//#include "kinect_v2_class.h"

using namespace std;
using namespace cv;

ros::Publisher topictakeoff;
ros::Publisher topiclanding;
ros::Publisher cmd_vel;
ros::Publisher topicinfo;
ros::Publisher emergency;
ros::ServiceClient serviceflattrim;


double velocity=0.5;
double strefa;
float maxX, minX, maxY, minY, maxZ, minZ, zerox, zeroy, zeroz;
float longitud, latitud, altitud;
int push_key_hand=0;
int push_key=0;
int marker1=0;

struct Data
{
    double var_strefa1, var_velocity1;
};
Data data;





void adjust(void);
void emergency_function(void);
void takeoff(void);
void land(void);
void forward(void);
void back(void);
void left(void);
void right(void);
void up(void);
void down(void);
void stop(void);
void turn(void);

void up_left(void);
void up_right(void);
void down_right(void);
void down_left(void);

void back_left(void);
void back_right(void);
void forward_left(void);
void forward_right(void);

void up_back(void);
void up_forward(void);
void down_back(void);
void down_forward(void);

geometry_msgs::Twist changeTwist(float x, float y, float z, float turn);

int getch();




#endif // HEADERS_H1
