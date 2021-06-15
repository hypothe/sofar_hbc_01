/****************************************//**
* \file gripper_routing.cpp
* \brief Node gripper control adapter
* \author Marco Gabriele Fedozzi (5083365@studenti.unige.it)
* \version 1.0
* \date 13/06/2021
*
* \details
*
* **Publishes to:**<BR>
*		`/robot/end_effector/left_gripper/gripper_action/goal` (control_msgs::GripperCommandActionGoal)<BR>
*		`/robot/end_effector/right_gripper/gripper_action/goal` (control_msgs::GripperCommandActionGoal)<BR>
*
* **Subscribes to:**<BR>
*		`robot/limb/left/left_gripper` (human_baxter_collaboration::BaxterGripperOpen)<BR>
*		`robot/limb/right/right_gripper` (human_baxter_collaboration::BaxterGripperOpen)<BR>
*
* Description:
*
* This node is an adapter, simply routing the
*	messages wrote by the finite_state_machine
*	to the simulation used to close the grippers
*	to those that the real Baxter controller
*	will need.
*
********************************************/

#include "ros/ros.h"

#include "human_baxter_collaboration/BaxterGripperOpen.h"
#include "control_msgs/GripperCommandActionGoal.h"



void routeCllbck(const human_baxter_collaboration::BaxterGripperOpenConstPtr& msg,
								std::shared_ptr<ros::Publisher> pub_
								)
{
	control_msgs::GripperCommandActionGoal out_msg;
	
	out_msg.goal.command.position = msg->open_gripper ? 100.0 : 0.0;
	pub_->publish(out_msg);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripper_router");
  ros::NodeHandle node_handle;
	
	std::shared_ptr<ros::Publisher> route_pubL, route_pubR;
  
  route_pubL = std::make_shared<ros::Publisher>(node_handle.advertise<control_msgs::GripperCommandActionGoal>
  								  							("/robot/end_effector/left_gripper/gripper_action/goal", 1000));
  route_pubR = std::make_shared<ros::Publisher>(node_handle.advertise<control_msgs::GripperCommandActionGoal>
  								  							("/robot/end_effector/right_gripper/gripper_action/goal", 1000));

	int tmp=10;
	std::function<void(const human_baxter_collaboration::BaxterGripperOpenConstPtr&)> leftCllbck =
								std::bind(routeCllbck, std::placeholders::_1, route_pubL);
								
	std::function<void(const human_baxter_collaboration::BaxterGripperOpenConstPtr&)> rightCllbck =
								std::bind(routeCllbck, std::placeholders::_1, route_pubR);
								
								
	ros::Subscriber local_clientL = node_handle.subscribe<human_baxter_collaboration::BaxterGripperOpen>("robot/limb/left/left_gripper", 1 ,
																	leftCllbck);
	ros::Subscriber local_clientR = node_handle.subscribe<human_baxter_collaboration::BaxterGripperOpen>("robot/limb/right/right_gripper", 1 ,
																	rightCllbck);


	ros::spin();
}
