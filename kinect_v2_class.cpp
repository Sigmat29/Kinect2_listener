#include "MyFunctions.h"

kinect_v2_class::kinect_v2_class()
{

}

void kinect_v2_class::reconfigure(kinect2_tracker::set_kinect_v2Config &config)
{
  data.var_strefa1 = config.strefa;
  data.var_velocity1 = config.velocity;
}
