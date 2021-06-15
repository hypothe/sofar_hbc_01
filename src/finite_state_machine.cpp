/****************************************//**
* \file finite_state_machine.cpp
* \brief Node setting up the environment for FSM class
* \author Marco Gabriele Fedozzi (5083365@studenti.unige.it)
* \version 1.0
* \date 14/06/2021
*
* \details
*
* **ServiceServer:**<BR>
*   `/baxter/collision_detection/<ARM>/result` (sofar_hbc_01::CollisionDetectionResult)<BR>
*
* **ServiceClient:**<BR>
*   `/block_to_pick` (sofar_hbc_01::Block2Pick)<BR>
*   `/empty_pos` (sofar_hbc_01::ClosestEmptySpace)<BR>
*   `/baxter/collision_detection/toggle` (sofar_hbc_01::CollisionDetectionToggle)<BR>
*
* **Publishes to:**<BR>
*		`/baxter_moveit_trajectory` (human_baxter_collaboration::BaxterTrajectory)<BR>
*		`robot/limb/<ARM>/<ARM>_gripper` (human_baxter_collaboration::BaxterGripperOpen)<BR>
*
* **Subscribes to:**<BR>
*		`/baxter_moveit_trajectory/result` (human_baxter_collaboration::BaxterTrajectoryResult)<BR>
*
* Description:
*
* This node is responsible for the whole state
*	machine implementation of the pick&place
*	skill of one individual arm.
*	The actual implementation is hidden
*	inside the `FSM` class, whilst this script
*	merely sets up the planning space and the
*	"Moveit!" move group interface.
*	Baxter is first of all encased in a confined
* workspace generating wall obstacles all
* around the table, in order to limit the
* movements RRTStar, the OMPL planner used,
*	can generate. Moreover, a set of "dynamic"
*	obstacles is generated, representing blocks
* covering half of the table, to be put in
*	in place when planning to move the eef in
*	order to always have it far above the table
*	surface, except when grasping blocks.
*
********************************************/
/*
	- in case of a failed trajectory replanning happens immediately, and not when the
		scene is considered free: might not be a problem in a non-slowly evolving scenario
	- do the grippers have enough time to open and close? Real implementation uses actions.
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
#include "human_baxter_collaboration/BaxterResultTrajectory.h"

#include "sofar_hbc_01/utils.h"
#include "sofar_hbc_01/FSM.h"
#include "sofar_hbc_01/Block2Pick.h"
#include "sofar_hbc_01/ClosestEmptySpace.h"

std::vector<double> table_pos, table_dim, table_extra = {0.0, 0.0, 0.1};
std::vector<double> block_dest;	//< this EEF destination position (either the middle of the table or the blue box)

std::vector<double> baxter_rest_pose_;	//< Baxter initial (at rest) Joint State

// MoveIt operates on sets of joints called "planning groups" and stores them in an object called
// the `JointModelGroup`. Throughout MoveIt the terms "planning group" and "joint model group"
// are used interchangably.
std::string ARM;
std::string PLANNING_GROUP;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;	std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
std::shared_ptr<FSM> fsm_;	//< The object implementing the FSM behavior


/* ------------------------ */
/****************************************//**
* Load parameters from the param server and
* initialize the move_group_interface.
*
********************************************/
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
	move_group_interface->setPlannerId("RRTStarkConfigState");
	 // RRTConnectkConfigDefault
	 // RRTStarkConfigState
	move_group_interface->setPlanningTime(1.5);
	move_group_interface->setNumPlanningAttempts(6);
	
  baxter_rest_pose_ = move_group_interface->getCurrentJointValues();
	
}


int main(int argc, char** argv)
{
	ros::ServiceClient client_b2p, 	client_ces;
	ros::Publisher traj_pub, gripper_pub;

  ros::init(argc, argv, "baxter_FSM");
  ros::NodeHandle node_handle;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  moveit_msgs::CollisionObject half_table_CO;

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(3);
  spinner.start();

  loadParam();
  
  const double back_margin = 0.6;
  // Add (extended)table
  std::vector<double> ext_pos = table_pos;
  std::vector<double> ext_dim = table_dim;
  ext_dim[0] += back_margin;
  ext_pos[0] -= back_margin/2.0;
  
  
  collision_objects.push_back(genBoxObst(move_group_interface->getPlanningFrame(), "table", ext_pos, ext_dim));
  
  
  const double lim_overst = 1.0;
  // Add ceiling
  std::vector<double> ceil_pos = ext_pos;
  std::vector<double> ceil_dim = ext_dim;
  ceil_dim[2] = 0.20;
  ceil_pos[2] += lim_overst + ext_dim[2]/2.0;
  
  collision_objects.push_back(genBoxObst(move_group_interface->getPlanningFrame(), "ceiling", ceil_pos, ceil_dim));
  
  
  // Add front wall
  std::vector<double> wall_pos = table_pos;
  std::vector<double> wall_dim = table_dim;
  wall_dim[0] = 0.2;	// 20cm
  wall_dim[2] += lim_overst;
  wall_pos[0] += table_dim[0]/2.0;	// superimposes a bit with the table, not an issue
  wall_pos[2] = wall_dim[2]/2.0;
  
  collision_objects.push_back(genBoxObst(move_group_interface->getPlanningFrame(), "front_wall", wall_pos, wall_dim));
  
  // Add side-wall
  std::vector<double> side_wall_pos = table_pos;
  std::vector<double> side_wall_dim = table_dim;
  side_wall_dim[0] += back_margin; // make it extend a bit beyond the table
  side_wall_pos[0] -= back_margin/2.0;
  side_wall_dim[1] = 0.2;	// 20cm
  side_wall_dim[2] += lim_overst;
  side_wall_pos[1] += (1-2*(int)(ARM=="right"))*(table_dim[1]/2.0 + side_wall_dim[1]/2.0);	// superimposes a bit with the table, not an issue
  side_wall_pos[2] = side_wall_dim[2]/2.0;
  
  collision_objects.push_back(genBoxObst(move_group_interface->getPlanningFrame(), std::string(ARM+"_side_wall"), side_wall_pos, side_wall_dim));
  
  // Add back wall
  std::vector<double> back_pos = wall_pos;
  std::vector<double> back_dim = wall_dim;
  back_pos[0] = ext_pos[0]-ext_dim[0]/2.0;	
  
  collision_objects.push_back(genBoxObst(move_group_interface->getPlanningFrame(), "back_wall", back_pos, wall_dim));
  
  
  // Now, let's add the "static" collision objects into the world
  // (using a vector that could contain additional objects)
  
  /*	NOTE: there's a convenient "setWorkspace()" func, but it turns out its supposed
  					to be used with mobile robots only, not high-dof arms.
 	*/
  planning_scene_interface->applyCollisionObjects(collision_objects);
  
  
  // Generate fictious obstacles to force the arm to stay far from the table
  std::vector<double> dyn_pos = ext_pos;
  std::vector<double> dyn_dim = ext_dim;
  dyn_dim[1] = table_dim[1]/2.0;
  dyn_dim[2] = 0.26;
  dyn_pos[1] += (1-2*(int)(ARM=="right"))*table_dim[1]/4.0;	// select the correct half-table
  dyn_pos[2] += table_dim[2]/2.0;	// superimposes a bit with the table, not an issue
  									
	half_table_CO = genBoxObst(move_group_interface->getPlanningFrame(), std::string(ARM+"_half_table"), dyn_pos, dyn_dim);
  
  fsm_ = std::make_shared<FSM>(
  						std::make_shared<ros::NodeHandle>(node_handle),
  						ARM,
							move_group_interface,
							planning_scene_interface
						);
  fsm_->setPickHeight(table_pos[2]+table_dim[2]/2.0+0.15); //< 15cm over the table
  fsm_->setBlockDest(block_dest);
  fsm_->setRestPose(baxter_rest_pose_);
  fsm_->setDynamicObst(half_table_CO);
  
  fsm_->init();

  ros::waitForShutdown();
  return 0;
}
