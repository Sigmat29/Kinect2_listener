#ifndef KINECT_V2_CLASS_H
#define KINECT_V2_CLASS_H
#include "MyFunctions.h"

class kinect_v2_class
{
public:
  kinect_v2_class();
void reconfigure(kinect2_tracker::set_kinect_v2Config &config);
};

#endif // KINECT_V2_CLASS_H
