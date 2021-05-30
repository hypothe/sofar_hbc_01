/* Author: Marco G. Fedozzi */

#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "human_baxter_collaboration/BaxterTrajectory.h"
#include "human_baxter_collaboration/BaxterStopTrajectory.h"
#include "human_baxter_collaboration/BaxterResultTrajectory.h"
#include "sofar_hbc_01/utils.h"
#include "sofar_hbc_01/Block.h"
#include "sofar_hbc_01/Block2Pick.h"
#include "sofar_hbc_01/ClosestEmptySpace.h"

enum states_ {START, REACH, PICK, RAISE, PLACE_BLUE, REMOVE_RED, END};

int state_ = START;
int next_state_ = START;
geometry_msgs::Pose goal_pose_;
geometry_msgs::Point block_grasp_offset_;

// MoveIt operates on sets of joints called "planning groups" and stores them in an object called
// the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
// are used interchangably.
std::string ARM;
std::string PLANNING_GROUP; // = ARM+"_arm";
moveit::planning_interface::MoveGroupInterface *move_group_interface;	moveit::planning_interface::PlanningSceneInterface *planning_scene_interface;

ros::ServiceClient client_b2p, client_ces;
ros::Publisher traj_pub;

std::shared_ptr<Block> block = NULL;

void loadParam(){

  if(!ros::param::get("~arm", ARM)){
  	ROS_ERROR("No parameter called '~arm' found.");
  	ros::shutdown();
  	return;
  }
  
  PLANNING_GROUP = ARM+"_arm";
  move_group_interface = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
	planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface;
	move_group_interface->setPlannerId("RRTConnectkConfigMechanical");
	move_group_interface->setPlanningTime(2);
	move_group_interface->setNumPlanningAttempts(4);
	
	block_grasp_offset_.x = 0;
	block_grasp_offset_.y = 0;
	block_grasp_offset_.z = 0.1;
	
}
void graspQuat(geometry_msgs::Quaternion* orig_orientation){
	tf2::Quaternion q_orig;
	// Get the original orientation of 'commanded_pose'
	tf2::convert(*orig_orientation, q_orig);
	// make eef perpendicular to the table
	q_orig.setRPY(M_PI, 0, M_PI);
	q_orig.normalize();
	// Stuff the new rotation back into the pose. This requires conversion into a msg type
	tf2::convert(q_orig, *orig_orientation);
}

void addTableObst(){
	moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_interface->getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "table";
	std::vector<double> table_pos, table_dim;
	if(!ros::param::get("table_pos", table_pos)){	ROS_ERROR("No parameter named 'table_pos' found");	}
	if(!ros::param::get("table_dim", table_dim)){	ROS_ERROR("No parameter named 'table_dim' found");	}

	// Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = table_dim[0];
  primitive.dimensions[primitive.BOX_Y] = table_dim[1];
  primitive.dimensions[primitive.BOX_Z] = table_dim[2];

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x = table_pos[0];
  table_pose.position.y = table_pos[1];
  table_pose.position.z = table_pos[2];

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

void publishPlan(geometry_msgs::Pose target_pose){
	move_group_interface->setPoseTarget(target_pose);
  move_group_interface->setGoalTolerance(0.01);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	// move_group_interface->execute(my_plan);

  ROS_INFO("Plan for pose goal %s", success ? "" : "FAILED");
  
  human_baxter_collaboration::BaxterTrajectory bxtr_traj;
  bxtr_traj.arm = ARM;
  bxtr_traj.trajectory.push_back(my_plan.trajectory_);
  traj_pub.publish(bxtr_traj);	// publish the plan trajectory
  
}

geometry_msgs::Pose offsetGoal(geometry_msgs::Pose goal_pose){
	geometry_msgs::Pose ogp = goal_pose;
	
	ogp.position.x += block_grasp_offset_.x;
	ogp.position.y += block_grasp_offset_.y;
	ogp.position.z += block_grasp_offset_.z;
	
	graspQuat(&(ogp.orientation));

	return ogp;

}

int FSM(int state){
	int next_state = state;
	geometry_msgs::Pose current_pose = move_group_interface->getCurrentPose().pose;
	geometry_msgs::Pose goal_pose;
	
	switch (state){
	
		case START:
		{
			// retrieve closest obj
			sofar_hbc_01::Block2Pick b2p;
			b2p.request.arm = ARM;
			b2p.request.eef_pose = current_pose;
			
			client_b2p.call(b2p);
			
			block = std::make_shared<Block>(b2p.response.block_name, b2p.response.block_color);
			block->setPose(b2p.response.block_pose);
			
			goal_pose = offsetGoal(block->getPose());
			
			publishPlan(goal_pose); // plan and publish it
			
			next_state = REACH;
			break;
		}
		case REACH:
		{
			// reach obj pose + offset
			//std::vector<double> current_orientation = move_group_interface->getCurrentRPY();
			//current_orientation[1] = 0;	// set eef perpendicular to the table (P = 0)
			if (block == NULL){
				ROS_ERROR("Status 'REACH' with no block grasped");
				next_state = START;
				break;
			}
			
			// go down to the block, being sure to correctly orient the eef
			geometry_msgs::Pose goal_pose = block->getPose();
			graspQuat(&(current_pose.orientation));
			
			publishPlan(goal_pose); // plan and publish it
			
			next_state = PICK;
			break;
		}
		case PICK:
		{
			// go down towards the obj
			// close the grippers
			next_state = RAISE;
		}
		case RAISE:
		{
			// raise the eef up of few cm
			// next_state = obj.isBlue() ? PLACE_BLUE : REMOVE_RED
			break;
		}
		case PLACE_BLUE:
		{
			// retrieve BlueBox pose
			// reach blue box + offset
			// release grippers
			next_state = START;
			break;
		}
		case REMOVE_RED:
		{
			// retrieve empty position
			// reach the position + offset
			// release grippers
			next_state = START;
			break;
		}
		case END:
		{
			// Do nothing for now
			// Should it (indirectly) terminate the node?
			break;
		}
	}
	
	return next_state;
}

/*	NOTE: instead of calling FSM directly we could call it in a loop from the main,
		making use of a flag to set whether a new call arrived from the last one:
		Can't really see any improvement there if not for the possibility to impose a
		rate for such check. Which, being this a slowly-evolving FSM, wouldn't bring any
		improvement I can think of.
*/
void resCllbck (const human_baxter_collaboration::BaxterResultTrajectory::ConstPtr& msg){
	if (msg->arm != ARM){ return; } // not meant for this node!
	
	if (msg->success){
		state_ = next_state_;	// the previous step has been successful, update the current state.
	}
	// if the update does not occour we're replanning for the same goal, but with the current
	// initial state.
	next_state_ = FSM(state_);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "baxter_FSM");
  ros::NodeHandle node_handle;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(3);
  spinner.start();
  
  ros::Subscriber res_sub = node_handle.subscribe("baxter_moveit_trajectory/result", 1, resCllbck);
  
  client_b2p = node_handle.serviceClient<sofar_hbc_01::Block2Pick>("/block_to_pick");
  client_ces = node_handle.serviceClient<sofar_hbc_01::ClosestEmptySpace>("/empty_pos");
  
  std::vector<std::shared_ptr<ros::ServiceClient> > vec_client;
  vec_client.push_back(std::make_shared<ros::ServiceClient>(client_b2p));
  vec_client.push_back(std::make_shared<ros::ServiceClient>(client_ces));
  
  waitForServices(vec_client);
  /*
  while(!client_b2p.exists() || ){
  	ROS_WARN("Service %s not found.", client_b2p.getService().c_str());
	  client_b2p.waitForExistence(ros::Duration(2));
  }
  while(!client_ces.exists()){
  	ROS_WARN("Service %s not found.", client_ces.getService().c_str());
	  client_ces.waitForExistence(ros::Duration(2));
  }
  */
  loadParam();
  addTableObst();
  
  /* Force bootstrap: */
  
	// publisher which pubs the trajectories w/ the info about the arm already in
  traj_pub = node_handle.advertise<human_baxter_collaboration::BaxterTrajectory>("/baxter_moveit_trajectory", 1000);
  
  /*
  while(ros::ok()){
		state = FSM(state);
  }
  */
  

  ros::waitForShutdown();
  return 0;
}
