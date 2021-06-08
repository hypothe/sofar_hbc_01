/* Author: Marco G. Fedozzi */

#include "ros/ros.h"

#include <vector>
#include <map>
#include <time.h>
#include <math.h>

#include "geometry_msgs/PoseStamped.h"
#include "sofar_hbc_01/ClosestEmptySpace.h"
#include "sofar_hbc_01/Block.h"
#include "sofar_hbc_01/BlocksPoses.h"
#include "sofar_hbc_01/utils.h"

const double max_dist = 0.04;

ros::ServiceClient client_blocks_tf;
	std::vector<geometry_msgs::Point> shift;

double dist2(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2){
	return std::sqrt(	std::pow(pose2.position.x - pose1.position.x, 2) + 
										std::pow(pose2.position.y - pose1.position.y, 2)	);
}

bool posCllbck(sofar_hbc_01::ClosestEmptySpace::Request &req, sofar_hbc_01::ClosestEmptySpace::Response &res){
	ROS_DEBUG("EMPTY SERVER CALLBACK");
	sofar_hbc_01::BlocksPoses bp;
	client_blocks_tf.call(bp);
	
	geometry_msgs::PoseStamped block_pose;
	geometry_msgs::Pose cell = req.eef_pose;
	bool empty = false;
	
	res.empty_pose = req.eef_pose; // init value
	
	for(auto s : shift){
		cell.position.x = req.eef_pose.position.x + s.x;
		cell.position.y = req.eef_pose.position.y + s.y;
		ROS_INFO("CELL_POS X(%lf) Y(%lf) Z(%lf)", cell.position.x,
																							cell.position.y,
																							cell.position.z);
		
		empty = true;
		for (auto block_pose : bp.response.blocks_poses){
			if(block_pose.header.frame_id == req.block_name) { continue; }
			if(dist2(block_pose.pose, cell) <= max_dist){
				empty = false;
				break;
			}
		}
		if(empty){
			res.empty_pose.position.x = cell.position.x;
			res.empty_pose.position.y = cell.position.y;
			break;
		}
	}
	return empty;
		
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "closest_empty_space");
  ros::NodeHandle node_handle;

 	// service that gets one EEF pose and returns the pose of the block closest to it
 	
 	//create the vector with the 9 cell
	geometry_msgs::Point point;
	point.x = 0.0; point.y = 0.0;
	shift.push_back(point);
	
	for(int i = -1; i <= 1; i++){
		for(int j = 1; j >= -1; j--){
			if (i==0 && j==0) continue;
			point.x = 0.05*i;
			point.y = 0.05*j;
			shift.push_back(point);
		}
	}
 	
	ros::ServiceServer service_closest_block = node_handle.advertiseService("/empty_pos", posCllbck);
	client_blocks_tf = node_handle.serviceClient<sofar_hbc_01::BlocksPoses>("/tf/blocks");
	
  
  ros::spin();
  return 0;
}

