/* Author: Marco G. Fedozzi */

/*
	Current limitations:
	- for the LEFT arm blocks are delivered always in the center of the Bluebox,
		not ideal in the simulation scenario (although in reality, since they're dropped
		and nod placed, they should roll over and not stack).
	- in case of a failed trajectory replanning happens immediately, and not when the
		scene is considered free: might not be a problem in a non-slowly evolving scenario
	- do the grippers have enough time to open and close?
*/

#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "human_baxter_collaboration/BaxterGripperOpen.h"
#include "human_baxter_collaboration/BaxterTrajectory.h"
// #include "human_baxter_collaboration/BaxterStopTrajectory.h"
#include "human_baxter_collaboration/BaxterResultTrajectory.h"
#include "sofar_hbc_01/utils.h"
#include "sofar_hbc_01/FSM.h"
#include "sofar_hbc_01/Block2Pick.h"
#include "sofar_hbc_01/ClosestEmptySpace.h"

//geometry_msgs::Pose BLOCK_DEST_;
std::map<std::string, bool> placed_;
std::vector<double> table_pos, table_dim, table_extra = {0.0, 0.0, 0.1};
std::vector<double> block_dest;

geometry_msgs::Pose baxter_rest_pose_;

// MoveIt operates on sets of joints called "planning groups" and stores them in an object called
// the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
// are used interchangably.
std::string ARM;
std::string PLANNING_GROUP; // = ARM+"_arm";
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;	std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
std::shared_ptr<FSM> fsm_;


/* ------------------------ */

void loadParam(){
	
  if(!ros::param::get("~arm", ARM)){
  	ROS_ERROR("No parameter called '~arm' found.");
  	ros::shutdown();
  	return;
  }
  if(!ros::param::get(std::string("block_dest_"+ARM), block_dest)){
  	ROS_ERROR("No parameter called 'block_dest_%s' found.", ARM.c_str());
  	ros::shutdown();
  	return;
  }
  
	if(!ros::param::get("table_pos", table_pos)){	ROS_ERROR("No parameter named 'table_pos' found");	}
	if(!ros::param::get("table_dim", table_dim)){	ROS_ERROR("No parameter named 'table_dim' found");	}
  
  PLANNING_GROUP = ARM+"_arm";
  move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
	planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
	move_group_interface->setPlannerId("RRTConnectkConfigMechanical");
	move_group_interface->setPlanningTime(2);
	move_group_interface->setNumPlanningAttempts(4);
	
  baxter_rest_pose_ = move_group_interface->getCurrentPose().pose;
	
}

void addBoxObst(std::string obst_name,
								std::vector<double> box_pos,
								std::vector<double> box_dim,
								std::vector<double> inflate = std::vector<double>(3, 0.0)){
								
	moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface->getPlanningFrame();

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

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  // (using a vector that could contain additional objects)
  ROS_INFO("Table added into the world");
  planning_scene_interface->applyCollisionObjects(collision_objects);
}

/*	NOTE: instead of calling FSM directly we could call it in a loop from the main,
		making use of a flag to set whether a new call arrived from the last one:
		Can't really see any improvement there if not for the possibility to impose a
		rate for such check. Which, being this a slowly-evolving FSM, wouldn't bring any
		improvement I can think of.
*/

int main(int argc, char** argv)
{
	ros::ServiceClient client_b2p, 	client_ces;
	ros::Publisher traj_pub, gripper_pub;

  ros::init(argc, argv, "baxter_FSM");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(3);
  spinner.start();

  loadParam();
  // Add table
  addBoxObst("table", table_pos, table_dim);
  // Add wall
  std::vector<double> wall_pos = table_pos;
  std::vector<double> wall_dim = table_dim;
  wall_dim[0] = 0.2;	// 20cm
  wall_dim[2] = 2*table_dim[2];
  wall_pos[0] += table_dim[0]/2;	// superimposes a bit with the table, not an issue
  wall_pos[2] += table_dim[2]/2;
  
  addBoxObst("human_wall", wall_pos, wall_dim);
  
  fsm_ = std::make_shared<FSM>(
  						std::make_shared<ros::NodeHandle>(node_handle),
  						ARM,
							move_group_interface,
							planning_scene_interface
						);
  fsm_->setPickHeight(table_pos[2]+table_dim[2]/2+0.15); //< 15cm over the table
  fsm_->setBlockDest(block_dest);
  fsm_->setRestPose(baxter_rest_pose_);
  
 // ROS_INFO("BAXTER_FSM_%s: ON", ARM.c_str());
  
  fsm_->init();

  ros::waitForShutdown();
  return 0;
}
