/****************************************//**
* \file closest_block.cpp
* \brief Node returning the *closest block to a given pose
* \author Marco Gabriele Fedozzi (5083365@studenti.unige.it)
* \version 1.0
* \date 03/06/2021
*
* \details
*
* **ServiceServer:**<BR>
*   `/block_to_pick` (sofar_hbc_01::Block2Pick)<BR>
*
* **ServiceClient:**<BR>
*   `/tf/blocks` (sofar_hbc_01::BlocksPoses)<BR>
*
*
* Description:
*
* This node returns the pose of a block to
* move the end effector toward.
* Despite the name, the position returned is
* not necessarily that of the closest blue
* block, but there is a simple priority
* system in place:
* 1. 	If the arm passed to the request is the
*			left one and there is a block near the
*			center line of the workspace, return
*			that block (since we want to free up the
*			"common space" as soon as possible);
*	2.	If there are unobstructed blue blocks
*			in the area served by the queries arm,
*			return the one closest to the current
*			arm pose;
*	3.	If there are only blue blocks obstructed
*			by red ones in the area served by the
*			queried arm, return the pose of the red
*			one covering the closest blue block;
*
********************************************/

#include "ros/ros.h"

#include <vector>
#include <map>

#include "geometry_msgs/PoseStamped.h"
#include "sofar_hbc_01/Block2Pick.h"
#include "sofar_hbc_01/BlocksPoses.h"
#include "sofar_hbc_01/Block.h"
#include "sofar_hbc_01/utils.h"

double pickThreshold = 0.05; //< max distance of a block from the center line to be considered still in the middle

ros::ServiceClient client_blocks_tf;

float table_height = -1;
std::vector<double> table_pos, table_dim;
float block_size = -1;

std::map<std::string, std::shared_ptr<Block> > blocks_; //< collection of the blocks (\sa Block),
																												//< with their name as key


/****************************************//**
* Service callback (`/block_to_pick`).
*
* \param req (sofar_hbc_01::Block2Pick::Request&):
*										the service request
*										eef_pose (geometry_msgs::Pose)<BR>
* \param res (sofar_hbc_01::Block2Pick::Response&):
*										the service response
*										
********************************************/
bool blockCllbck(sofar_hbc_01::Block2Pick::Request &req, sofar_hbc_01::Block2Pick::Response &res){

	// retrieve current end effector pose
	std::vector<std::shared_ptr<Block> > v_blue, v_red, v_all;
	std::map<std::string, bool> placed;
	std::map<std::string, bool> grasped;
	std::vector<double> block_dest;
	geometry_msgs::Pose block_dest_pose;
	
	
	if(!ros::param::get(std::string("block_grasped"), grasped)){	ROS_ERROR("No parameter named 'block_grasped' found");	}
	if(!ros::param::get(std::string("block_dest_"+req.arm), block_dest)){	ROS_ERROR("No parameter named 'block_dest_%s' found", req.arm.c_str());	}
	block_dest_pose.position.x = block_dest[0];	block_dest_pose.position.y = block_dest[1];	block_dest_pose.position.z = block_dest[2];
	
	sofar_hbc_01::BlocksPoses bp;
	client_blocks_tf.call(bp);
	
	geometry_msgs::PoseStamped block_pose;
	std::string block_name;
	std::shared_ptr<Block> block;
	
	// separate in blue and red blocks those not yet placed
	for (auto block_pose : bp.response.blocks_poses){

		block_name = block_pose.header.frame_id;
		if (!blocks_.count(block_name)) continue; // no element with such name found
		
		block = blocks_[block_name];
		
		// do not consider already placed blocks;
		if (placed[block_name] || grasped[block_name]){	continue;	}
		// do not consider blocks in the unreachable half-table for this arm
		if (	(req.arm == "right" && block_pose.pose.position.y > 0 + pickThreshold) ||
					(req.arm == "left" && block_pose.pose.position.y < 0 - pickThreshold)		){	continue;	}
					
		block->setPose(block_pose.pose);
		block->setObstruction("");
		// Dynamic placement check
		if (dist2(block->getPose(), block_dest_pose) <= 2*pickThreshold){
			placed[block->getName()] = true;
			continue; // skip if it's already placed
		}
		
		v_all.push_back(block);
		if (block->isBlue())	{ v_blue.push_back(block);	}
		else									{ v_red.push_back(block);	}
	}
	// Update the placement status
	ros::param::set(std::string("block_placed_"+req.arm), placed);
	
	// check for obstruction
	for (auto o_block : v_all){
		if (o_block->getPose().position.z > table_height + block_size){
		
			for (auto blue_block : v_blue){
				if (o_block->getPose().position.z > blue_block->getPose().position.z && 
						dist2(blue_block->getPose(), o_block->getPose()) < block_size){
						
						blue_block->setObstruction(o_block->getName());
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
										abs(blue_block->getPose().position.y - table_pos[1]) <= pickThreshold &&
										abs(blue_block->getPose().position.x - table_pos[0]) < table_dim[0]/2.0
										);
		if (middlePlaced){
			ROS_INFO("Block %s in the MIDDLE", blue_block->getName().c_str());
		}
		
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
	else if (!obst_closest_block_name.empty()){
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
  
  std::vector<std::shared_ptr<ros::ServiceClient> > vec_client;
  vec_client.push_back(std::make_shared<ros::ServiceClient>(client_blocks_tf));
  
  waitForServices(vec_client);
  // service that gets one EEF pose and returns the pose of the block closest to it
	ros::ServiceServer service_closest_block = node_handle.advertiseService("/block_to_pick", blockCllbck);
	
  
  ros::spin();
  return 0;
}

