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
#include "human_baxter_collaboration/BaxterGripperOpen.h"
#include "human_baxter_collaboration/BaxterTrajectory.h"

// typedef int state_t
enum state_t {START, REACH, PICK, RAISE, PLACE_BLUE, REMOVE_RED, ERR, END, IDLE};

class FSM{

	public:
		static std::shared_ptr<FSM> getFSM(
					std::shared_ptr<ros::NodeHandle> node_handle,
					std::string ARM,
					std::shared_ptr<ros::ServiceClient> client_b2p, 	std::shared_ptr<ros::ServiceClient> client_ces,
					std::shared_ptr<ros::Publisher> traj_pub,				std::shared_ptr<ros::Publisher> gripper_pub,
					std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
					std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface
					);
		void setPickHeight(double n_height);
		void setBlockDest(std::vector<double> block_dest);
		void setRestPose(geometry_msgs::Pose rest_pose);
		void trajectoryResult(bool success);
		void init();
		/*{
			evolve = true;
			FSM_clock = node_handle->createTimer(ros::Duration(1.0), &FSM::clock_Cllbck, this);
		}*/
	

	private:
		
		void graspQuat(geometry_msgs::Quaternion* orig_orientation);
		geometry_msgs::Pose offsetGoal(geometry_msgs::Pose goal_pose);
		double getClosestBlock(geometry_msgs::Pose current_pose);
		void setBlockGrasped(std::string name, bool grasp);
		
		bool generatePlan			(	geometry_msgs::Pose target_pose);
		bool generateCartesian(	geometry_msgs::Pose target_pose);
		
		bool stateEvolution();

		void clock_Cllbck(const ros::TimerEvent&);
				
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
		// bool idle;
		bool evolve;
		// std::thread th_trajectoryResult; //< shouldn't be necessary
		geometry_msgs::Pose current_pose;
		geometry_msgs::Pose goal_pose;
		geometry_msgs::Pose BLOCK_DEST_;
		geometry_msgs::Pose baxter_rest_pose_;
		const double maxInterBlockDist = 0.01;
		double pick_height;
		geometry_msgs::Point block_grasp_offset_;
		std::shared_ptr<moveit_msgs::RobotTrajectory> trajectory;
		
		std::shared_ptr<Block> block;
		std::shared_ptr<ros::NodeHandle> node_handle;
		std::string ARM;
		std::shared_ptr<ros::ServiceClient> client_b2p,	client_ces;
		std::shared_ptr<ros::Publisher> traj_pub, gripper_pub;
		std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
		std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;

	//protected:	//< should be protected in order to implement a singleton
								// for some reason ROS callback do not like that, seems
								// they try to build a new class instance, or in any case
								// accessing the constructor
	public:
		static std::shared_ptr<FSM> fsm_;
		FSM(	std::shared_ptr<ros::NodeHandle> node_handle,
					std::string ARM,
					std::shared_ptr<ros::ServiceClient> client_b2p, 	std::shared_ptr<ros::ServiceClient> client_ces,
					std::shared_ptr<ros::Publisher> traj_pub,				std::shared_ptr<ros::Publisher> gripper_pub,
					std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
					std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface
				);
	
};

std::shared_ptr<FSM> FSM::fsm_= nullptr;

std::shared_ptr<FSM> FSM::getFSM(
					std::shared_ptr<ros::NodeHandle> node_handle,
					std::string ARM,
					std::shared_ptr<ros::ServiceClient> client_b2p, 	std::shared_ptr<ros::ServiceClient> client_ces,
					std::shared_ptr<ros::Publisher> traj_pub,				std::shared_ptr<ros::Publisher> gripper_pub,
					std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
					std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface
					)
{
	if (fsm_ == nullptr){
		fsm_ = std::make_shared<FSM>(
					node_handle,
					ARM,
					client_b2p, client_ces,
					traj_pub, gripper_pub,
					move_group_interface,
					planning_scene_interface
			);
	}
	return fsm_;				
}

#endif
