/****************************************//**
* \file FSM.h
* \brief Class definition of the FSM behavior
* \author Marco Gabriele Fedozzi (5083365@studenti.unige.it)
* \version 1.0
* \date 14/06/2021
*
********************************************/
#ifndef FSM_H
#define FSM_H

#include "ros/ros.h"

#include <memory>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "sofar_hbc_01/Block.h"
#include "sofar_hbc_01/utils.h"
#include "sofar_hbc_01/CollisionDetectionToggle.h"
#include "sofar_hbc_01/CollisionDetectionResult.h"
#include "human_baxter_collaboration/BaxterGripperOpen.h"
#include "human_baxter_collaboration/BaxterTrajectory.h"
#include "human_baxter_collaboration/BaxterResultTrajectory.h"

enum state_t {START, REACH, PICK, RAISE, PLACE_BLUE, REMOVE_RED, ERR, END, IDLE};

/****************************************//**
* This class receives the defintion of both a
*	"Moveit!" move group interface and a
*	planning scene, together with a node handle.
*	It takes care of all communications as well
*	as the algorithmic implementation of the FSM.
*
********************************************/
class FSM{

	public:
		/****************************************//**
		* FSM constructor
		*
		*	Aside from storing the passed parameters it
		*	also initializes all communication endopoints,
		*	the starting state and the inner clock.
		*
		*	\param node_handle (std::shared_ptr<ros::NodeHandle>)
		*	\param ARM (std::string)
		*	\param move_group_interface (std::shared_ptr<moveit::planning_interface::MoveGroupInterface>)
		*	\param planning_scene_interface (std::shared_ptr<moveit::planning_interface::PlanningSceneInterface>)
		*
		********************************************/
		FSM(	std::shared_ptr<ros::NodeHandle> node_handle,
					std::string ARM,
					std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
					std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface
				);
		
		/****************************************//**
		* Set the height above blocks for the EEf
		* before picking.
		*
		*	If this is not called the default one will
		* be used.
		*
		*	\param n_height (double)
		*
		********************************************/
		void setPickHeight(double n_height);
		
		/****************************************//**
		* Set the block destination
		*
		*	A vector of three elements is expected.
		*
		*	\param block_dest (std::vector<double>)
		*
		*	\throws std::invalid_argument
		*
		********************************************/
		void setBlockDest(std::vector<double> block_dest);
		
		/****************************************//**
		* Set the arm rest state joint values
		*
		*	A vector of seven elements is expected.
		*
		*	\param block_dest (std::vector<double>)
		*
		*	\throws std::invalid_argument
		*
		********************************************/
		void setRestPose(std::vector<double> rest_pose);
		
		/****************************************//**
		* Save a dynamic obstacle to use for planning
		*
		*	The obstacle is implicitly expected to be
		*	something that enforces a certain behavior
		*	while planning for all goal poses.
		* Note it won't be used for carthesian plannings.
		*
		*	\param dynamic_obst (moveit_msgs::CollisionObject)
		*
		********************************************/
		void setDynamicObst(moveit_msgs::CollisionObject dynamic_obst);
		
		/****************************************//**
		* Start the FSM behavior
		*
		*	Services server are waited for before starting
		*	the inner clock responsible for the FSM.
		*
		********************************************/
		void init();

	private:
		
		void graspQuat(geometry_msgs::Quaternion* orig_orientation);
		geometry_msgs::Pose offsetGoal(geometry_msgs::Pose goal_pose);
		double getClosestBlock(geometry_msgs::Pose current_pose);
		void setBlockGrasped(std::string name, bool grasp);
		void gripperOpen(bool open);
		
		void applyDynamicCollisionObjects();
		void removeDynamicCollisionObjects();
		
		bool generatePlan(std::vector<double> joint_target);
		bool generatePlan			(	geometry_msgs::Pose target_pose);
		bool computeTraj(moveit_msgs::RobotTrajectory& traj);
		bool generateCartesian(	geometry_msgs::Pose mid_pose, 
														geometry_msgs::Pose target_pose);
		
		bool stateEvolution();

		void clock_Cllbck(const ros::TimerEvent&);
		bool collisionPolicy(	sofar_hbc_01::CollisionDetectionResult::Request &req,
													sofar_hbc_01::CollisionDetectionResult::Response &res);
		void trajectoryResult(const human_baxter_collaboration::BaxterResultTrajectory::ConstPtr& msg);
		void resetCollisionWait();
		void setCollisionWait();
				
		state_t start();
		state_t reachBlock();
		state_t pickBlock();
		state_t raiseBlock();
		state_t placeBlue();
		state_t removeRed();
		state_t	rest();
		
		
		int BAXTER_ATTEMPTS_;
		const int MAX_BAXTER_ATTEMPTS_ = 10;
		ros::Timer FSM_clock;
		state_t state;
		state_t next_state;
		bool evolve;
		double collision_wait;
		geometry_msgs::Pose current_pose;
		geometry_msgs::Pose goal_pose;
		geometry_msgs::Pose BLOCK_DEST_;
		std::vector<double> baxter_rest_pose_;
  	std::vector<moveit_msgs::CollisionObject> dynamic_obst;
		const double maxInterBlockDist = 0.01;
		double pick_height;
		geometry_msgs::Point block_grasp_offset_;
		std::shared_ptr<moveit_msgs::RobotTrajectory> trajectory;
		
		std::shared_ptr<Block> block;
		std::shared_ptr<ros::NodeHandle> node_handle;
		std::string ARM;
		std::shared_ptr<ros::ServiceClient> client_b2p,	client_ces, client_cd;
		std::shared_ptr<ros::ServiceServer> collision_result_srv;
		std::shared_ptr<ros::Publisher> traj_pub, gripper_pub;
		std::shared_ptr<ros::Subscriber> res_sub;
		std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
		std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;
};

#endif
