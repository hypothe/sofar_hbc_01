/****************************************//**
* \file utils.cpp
* \brief Utility functions implementation
* \author Marco Gabriele Fedozzi (5083365@studenti.unige.it)
* \version 1.0
* \date 14/06/2021
*
* \details
*
* Description:
*
* This script implements the utility functions
*	defined at utils.h
*
********************************************/

#include "sofar_hbc_01/utils.h"

void waitForServices (std::vector<std::shared_ptr<ros::ServiceClient> > clients)
{
	bool ready=false;
	
	while(!ready){
		ready = true;
		for(auto client : clients){
			if (!client->exists()){
				ROS_WARN("Service %s not found.", client->getService().c_str());
				client->waitForExistence(ros::Duration(1));
				ready = false;
			}
		}
	}
	return;
}


double dist2(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
	return std::sqrt(	std::pow(pose2.position.x - pose1.position.x, 2) + 
										std::pow(pose2.position.y - pose1.position.y, 2)	);
	
}
double dist3(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
	return std::sqrt(	std::pow(pose2.position.x - pose1.position.x, 2) + 
										std::pow(pose2.position.y - pose1.position.y, 2) + 
										std::pow(pose2.position.z - pose1.position.z, 2)	);
	
}
double dist3(fcl::Vector3f vec1, fcl::Vector3f vec2)
{	
	return std::sqrt(	std::pow(vec1[0] - vec2[0], 2) + 
										std::pow(vec1[1] - vec2[1], 2) + 
										std::pow(vec1[2] - vec2[2], 2)	);
	
}



moveit_msgs::CollisionObject genBoxObst(
								std::string frame_id,
								std::string obst_name,
								std::vector<double> box_pos,
								std::vector<double> box_dim,
								std::vector<double> inflate)
{
								
	moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;

  // The id of the object is used to identify it.
  collision_object.id = obst_name;
	std::vector<double> table_pos, table_dim;

	// Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = box_dim[0] + 2*inflate[0];
  primitive.dimensions[primitive.BOX_Y] = box_dim[1] + 2*inflate[1];
  primitive.dimensions[primitive.BOX_Z] = box_dim[2] + 2*inflate[2]; //< added something... maybe replace with block dim

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x = box_pos[0];
  table_pose.position.y = box_pos[1];
  table_pose.position.z = box_pos[2];

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(table_pose);
  collision_object.operation = collision_object.ADD;

	ROS_DEBUG("Obstacle '%s' generated.", obst_name.c_str());
	
	return collision_object;
}

