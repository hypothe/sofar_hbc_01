/* Author: Marco G. Fedozzi */

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

// typedef int state_t
enum state_t {START, REACH, PICK, RAISE, PLACE_BLUE, REMOVE_RED, ERR, END, IDLE};

class FSM{

	public:
		FSM(	std::shared_ptr<ros::NodeHandle> node_handle,
					std::string ARM,
					std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
					std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface
				);
		void setPickHeight(double n_height);
		void setBlockDest(std::vector<double> block_dest);
		void setRestPose(std::vector<double> rest_pose);
		void setDynamicObst(moveit_msgs::CollisionObject dynamic_obst);
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
		// std::thread th_trajectoryResult; //< shouldn't be necessary
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
