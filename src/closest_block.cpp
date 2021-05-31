/* Author: Marco G. Fedozzi */

#include "ros/ros.h"

#include <vector>
#include <map>

#include "geometry_msgs/PoseStamped.h"
#include "sofar_hbc_01/Block2Pick.h"
#include "sofar_hbc_01/BlocksPoses.h"
#include "sofar_hbc_01/Block.h"
#include "sofar_hbc_01/utils.h"

double pickThreshold = 0.05;

ros::ServiceClient client_blocks_tf;

float table_height = -1;
std::vector<double> table_pos, table_dim;
float block_size = -1;

std::map<std::string, std::shared_ptr<Block> > blocks_;


double dist2(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2){
	return std::sqrt(	std::pow(pose2.position.x - pose1.position.x, 2) + 
										std::pow(pose2.position.y - pose1.position.y, 2)	);
	
}
double dist3(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2){
	return std::sqrt(	std::pow(pose2.position.x - pose1.position.x, 2) + 
										std::pow(pose2.position.y - pose1.position.y, 2) + 
										std::pow(pose2.position.z - pose1.position.z, 2)	);
	
}

bool blockCllbck(sofar_hbc_01::Block2Pick::Request &req, sofar_hbc_01::Block2Pick::Response &res){

	// retrieve current end effector pose
	std::vector<std::shared_ptr<Block> > v_blue, v_red;
	std::map<std::string, bool> placed;
	
	if(!ros::param::get("block_placed", placed)){	ROS_ERROR("No parameter named 'block_placed' found");	}
	
	sofar_hbc_01::BlocksPoses bp;
	client_blocks_tf.call(bp);
	
	//int n_blocks = bp.response.blocks_poses.size;
	geometry_msgs::PoseStamped block_pose;
	std::string block_name;
	std::shared_ptr<Block> block;
	
	// separate in blue and red blocks those not yet placed
	//for (int i=0; i<n_blocks; i++){
	for (auto block_pose : bp.response.blocks_poses){

		block_name = block_pose.header.frame_id;
		if (!blocks_.count(block_name)) continue; // no element with such name found
		
		// block = std::make_shared<Block>(blocks_[block_name]);
		block = blocks_[block_name];
		
		block->setPlaced(placed[block_name]);
		
		// do not consider already placed blocks;
		if (placed[block_name]){	continue;	}
		// do not consider blocks in the unreachable half-table for this arm
		if (	(req.arm == "right" && block_pose.pose.position.y > 0 + pickThreshold) ||
					(req.arm == "left" && block_pose.pose.position.y < 0 - pickThreshold)		){	continue;	}
					
		block->setPose(block_pose.pose);
		block->setObstruction("");
		
		
		if (block->isBlue())	{ v_blue.push_back(block);	}
		else									{ v_red.push_back(block);	}
	}
	
	// check for obstruction
	for (auto red_block : v_red){
		if (red_block->getPose().position.z > table_height + block_size){
		
			for (auto blue_block : v_blue){
				if (red_block->getPose().position.z > blue_block->getPose().position.z && 
						dist2(blue_block->getPose(), red_block->getPose()) < block_size){
						
						blue_block->setObstruction(red_block->getName());
				}
			}
		}
	}
	
	// compute distance for unobstructed blue blocks
	double min_dist = 5000.0; // 5 kilometers, pretty far away
	double obst_min_dist = min_dist;
	double dist;
	std::string closest_block_name = "", obst_closest_block_name = "";
	geometry_msgs::Pose eef_pose = req.eef_pose;
	bool middlePlaced;
	
	for (auto blue_block : v_blue){
		// if (blue_block->getPlaced()){	continue;	} // <- already checked

		// give absolute precedence to any blue block in the middle of the table
		// note we're not specifying the MiddlePoint, but anywhere in the middle of the table
		// Note also the check on x distance, just to be sure the block is on the table.
		//	Dirty, but should work.
		middlePlaced = (req.arm == "left" && 
										abs(blue_block->getPose().position.y - table_pos[1]) <= block_size &&
										abs(blue_block->getPose().position.x - table_pos[0]) < table_dim[0]/2
										);
		
		dist = dist3(eef_pose, blue_block->getPose());
		// 
		if (blue_block->getObstructedBy().empty() && (dist < min_dist || middlePlaced))
		{
			// save the closest unobstructed blue block
			min_dist = dist;
			closest_block_name = blue_block->getName();
		}
		else if(!blue_block->getObstructedBy().empty() && (dist < obst_min_dist || middlePlaced))
		{
			// save the closest red block obstructing a blue one
			//
			obst_min_dist = dist;
			obst_closest_block_name = blue_block->getObstructedBy();
		}
		
		if (middlePlaced) break;
	}
	
	if (!closest_block_name.empty()){
		// if there was at least an unobstructed blue block
		block = blocks_[closest_block_name];
	}
	else if (!closest_block_name.empty()){
		// if there wasn't at least an unobstructed blue block
		// but at least an obstructed one
		block = blocks_[obst_closest_block_name];
	}
	else{
		// Notice the call returns false if no blue block can be picked
		block = std::make_shared<Block>(std::string("END"), std::string("END"));
		return false;
	}
  
  res.block_name = block->getName();
  res.block_color = block->getColor();
  res.block_pose = block->getPose();
  
  return true;
}

void load_params(){

	std::map<std::string, std::string> blocks_colors;
	if(!ros::param::get("blocks_colors", blocks_colors)){	ROS_ERROR("No parameter named 'blocks_color' found");	}
	if(!ros::param::get("blocks_dim", block_size)){	ROS_ERROR("No parameter named 'blocks_dim' found");	}
	
	if(!ros::param::get("table_pos", table_pos)){	ROS_ERROR("No parameter named 'table_pos' found");	}
	if(!ros::param::get("table_dim", table_dim)){	ROS_ERROR("No parameter named 'table_dim' found");	}
	table_height = table_pos[2] + table_dim[2]/2;
	
	// instantiate the blocks
	for (std::pair<std::string, std::string> bc : blocks_colors){
		blocks_[bc.first] = std::make_shared<Block>(bc.first, bc.second);
	}
	
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "closest_block");
  ros::NodeHandle node_handle;
  
	// load parameters
	load_params();
	
	// client that retrieves the current blocks tf
  client_blocks_tf = node_handle.serviceClient<sofar_hbc_01::BlocksPoses>("/tf/blocks");
  /*while(!client_blocks_tf.exists()){
  	ROS_WARN("Service %s not found.", client_blocks_tf.getService().c_str());
	  client_blocks_tf.waitForExistence(ros::Duration(2));
  }*/
  
  std::vector<std::shared_ptr<ros::ServiceClient> > vec_client;
  vec_client.push_back(std::make_shared<ros::ServiceClient>(client_blocks_tf));
  
  waitForServices(vec_client);
  // service that gets one EEF pose and returns the pose of the block closest to it
	ros::ServiceServer service_closest_block = node_handle.advertiseService("/block_to_pick", blockCllbck);
	
  
  ros::spin();
  return 0;
}

