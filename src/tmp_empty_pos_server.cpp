/* Author: Marco G. Fedozzi */

#include "ros/ros.h"

#include <vector>
#include <map>
#include <time.h>
#include <math.h>

#include "geometry_msgs/PoseStamped.h"
#include "sofar_hbc_01/ClosestEmptySpace.h"
#include "sofar_hbc_01/Block.h"
#include "sofar_hbc_01/utils.h"

const double max_dist = 0.1;
const double min_dist = 0.05;

bool posCllbck(sofar_hbc_01::ClosestEmptySpace::Request &req, sofar_hbc_01::ClosestEmptySpace::Response &res){
	double ang = double(rand())/double(RAND_MAX) * 90.0;
	double dist = double(rand())/double(RAND_MAX) * (max_dist - min_dist) + min_dist;
	
	if (req.arm == "left"){
		ang = 90.0 + ang * M_PI / 180.0;
	}
	else{
		ang = (270.0 + ang) * M_PI / 180.0;
	}
	res.empty_pose = req.eef_pose;
	res.empty_pose.position.x += dist*cos(ang);
	res.empty_pose.position.y += dist*sin(ang);
  
  return true;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "closest_empty_space");
  ros::NodeHandle node_handle;
  
  srand(time(NULL));

  // service that gets one EEF pose and returns the pose of the block closest to it
	ros::ServiceServer service_closest_block = node_handle.advertiseService("/empty_pos", posCllbck);
	
  
  ros::spin();
  return 0;
}

