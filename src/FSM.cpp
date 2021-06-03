#include "sofar_hbc_01/FSM.h"
#include "sofar_hbc_01/Block2Pick.h"
#include "sofar_hbc_01/ClosestEmptySpace.h"
#include <stdexcept>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

FSM::FSM(	std::shared_ptr<ros::NodeHandle> node_handle,
					std::string ARM,
					std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
					std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface
				)
	:	node_handle(node_handle), ARM(ARM),
		move_group_interface(move_group_interface),
		planning_scene_interface(planning_scene_interface)
{
	BAXTER_ATTEMPTS_ = 0;
  
  state = START;
  next_state = START;
  block = nullptr;
  pick_height = 0.0;
  
	block_grasp_offset_.x = 0;
	block_grasp_offset_.y = 0;
	block_grasp_offset_.z = 0.15; // 15 cm upward offset
	
	// publisher which pubs the trajectories w/ the info about the arm already in
  traj_pub = std::make_shared<ros::Publisher>
  						(node_handle->advertise<human_baxter_collaboration::BaxterTrajectory>
  							("/baxter_moveit_trajectory", 1000)
  						);
  gripper_pub = std::make_shared<ros::Publisher>
  						(node_handle->advertise<human_baxter_collaboration::BaxterGripperOpen>
  							(std::string("/robot/limb/"+ARM+"/"+ARM+"_gripper"), 1000)
  						);
  
  // WARN: do not set 1 as subscriber queue dimension or you could lose the message, since it could 
  // be overwritten by the one meant for the other arm!
  res_sub = std::make_shared<ros::Subscriber>
  						(node_handle->subscribe("baxter_moveit_trajectory/result", 10, &FSM::trajectoryResult, this));
  
  client_b2p = std::make_shared<ros::ServiceClient>
  							(node_handle->serviceClient<sofar_hbc_01::Block2Pick>
  								("/block_to_pick")
  							);
  client_ces = std::make_shared<ros::ServiceClient>
  							(node_handle->serviceClient<sofar_hbc_01::ClosestEmptySpace>
  								("/empty_pos")
  							);
  FSM_clock = node_handle->createTimer(ros::Duration(1.0), &FSM::clock_Cllbck, this, false, false);
}
// we need the publisher and the two clients

void FSM::init()
{
	waitForServices(std::vector<std::shared_ptr<ros::ServiceClient> > {client_b2p, client_ces});
	FSM_clock.start();
	evolve = true;
	ROS_INFO("BAXTER_FSM_%s: ON", ARM.c_str());
}

void FSM::setPickHeight(double n_height){
	if (n_height <= 0.0){	throw std::invalid_argument("Expected height > 0.0"); }
	pick_height = n_height;
}
void FSM::setBlockDest(std::vector<double> block_dest)
{
	if (block_dest.size() < 3){	throw std::invalid_argument("Expected 3 values");	}

	BLOCK_DEST_.position.x = block_dest[0];
	BLOCK_DEST_.position.y = block_dest[1];
	BLOCK_DEST_.position.z = block_dest[2];
	BLOCK_DEST_.orientation.y = 1;
}

void FSM::setRestPose(geometry_msgs::Pose rest_pose){
		baxter_rest_pose_ = rest_pose;
}

void FSM::graspQuat(geometry_msgs::Quaternion* goal_orientation){
	tf2::Quaternion q_orig, q_goal, q_rel, q_inv;
	tf2::convert(*goal_orientation, q_goal);
	tf2::convert(current_pose.orientation, q_orig);
	q_inv = q_orig;
	q_inv[3] = - q_orig[3]; //< inverse
	q_rel = q_goal*q_inv;
	q_rel.normalize();
	
	double r,p,y;
	tf2::Matrix3x3 mat(q_rel);
	mat.getRPY(r,p,y);
	//std::vector<double> cRPY = move_group_interface->getCurrentRPY();
	
	ROS_DEBUG("%s RELATIVEy: %lf", ARM.c_str(), y);
	// allow for perpendicular grasps
	while (abs(y - M_PI/2) <= M_PI/4){y -= M_PI/2;}
	while (abs(y + M_PI/2) <= M_PI/4){y += M_PI/2;}
	
	q_rel.setRPY(r,p,y);  // Calculate the new orientation
	q_goal = q_rel*q_orig;
	q_goal.normalize();

	tf2::Matrix3x3 mat_g(q_goal);
	mat_g.getRPY(r,p,y);
	q_goal.setRPY(r, 0.0, y);
	q_goal.normalize();
	
	tf2::convert(q_goal, *goal_orientation);
	/*
	orig_orientation->x = 0.0;
	orig_orientation->y = 1.0;
	orig_orientation->z = 0.0;
	orig_orientation->w = 0.0;
	*/
}

geometry_msgs::Pose FSM::offsetGoal(geometry_msgs::Pose goal_pose){
	geometry_msgs::Pose ogp = goal_pose;
	
	ogp.position.x += block_grasp_offset_.x;
	ogp.position.y += block_grasp_offset_.y;
	ogp.position.z = pick_height>0.0 ? pick_height : ogp.position.z + block_grasp_offset_.z;
	
	//graspQuat(&(ogp.orientation));
	ogp.orientation.x = 0.0;
	ogp.orientation.y = 1.0;
	ogp.orientation.z = 0.0;
	ogp.orientation.w = 0.0;

	return ogp;
}
double FSM::getClosestBlock(geometry_msgs::Pose current_pose){
		
	sofar_hbc_01::Block2Pick b2p;
	b2p.request.arm = ARM;
	b2p.request.eef_pose = current_pose;
			
	// if call returns false means no obj was retrieved
	// the table is shy of blue blocks
	if (!client_b2p->call(b2p)){	return -1.0;	}
			
	block = std::make_shared<Block>(b2p.response.block_name, b2p.response.block_color);
	block->setPose(b2p.response.block_pose);
	
	return dist3(current_pose, block->getPose());
}
void FSM::setBlockGrasped(std::string name, bool grasp){
	std::map<std::string, bool> grasped;
	
	if(!ros::param::get("block_grasped", grasped)){
		ROS_ERROR("No parameter called 'block_grasped' found.");
	}
	if (!grasped.count(name)){return;}
	grasped[name] = grasp;
	ros::param::set("block_grasped", grasped);
	return;
}
/*	--------------	*/
/*	-- PLANNING --	*/

bool FSM::generatePlan(	geometry_msgs::Pose target_pose)
{

	move_group_interface->setPoseTarget(target_pose);
  move_group_interface->setGoalTolerance(0.005);
  int attempt = 0;
  const int max_attempts = 10;
  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group_interface
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  bool success = false;
  
  while (!success && attempt < max_attempts){
  	success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  	ROS_INFO("Plan for pose goal %s %s", ARM.c_str(), success ? "" : "FAILED");
  	ros::Duration(1.0).sleep();
  	attempt++;
  }
	if (attempt >= max_attempts){
		return false;
	}
	
	ROS_DEBUG("PLANNING EXECUTED");
	// move_group_interface->execute(my_plan);
	trajectory = std::make_shared<moveit_msgs::RobotTrajectory>(my_plan.trajectory_);
	
  return true;
}

bool FSM::generateCartesian(geometry_msgs::Pose mid_pose, geometry_msgs::Pose target_pose)
{
  moveit_msgs::RobotTrajectory c_trajectory;
  std::vector<geometry_msgs::Pose> waypoints;
  double eef_step = 0.15, jump_threshold = 0.0;
	waypoints.push_back(current_pose);
	////// 
	/* 
  geometry_msgs::Pose init_rot;
  init_rot.position.x = current_pose.position.x;
  init_rot.position.y = current_pose.position.y;
  init_rot.position.z = current_pose.position.z;
  
  init_rot.orientation.x = target_pose.orientation.x;
  init_rot.orientation.y = target_pose.orientation.y;
  init_rot.orientation.z = target_pose.orientation.z;
  init_rot.orientation.w = target_pose.orientation.w;
  */
  //////
	waypoints.push_back(mid_pose);
	waypoints.push_back(target_pose);
	double fraction = move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, c_trajectory);
  
  ROS_DEBUG("CARTESIAN FRACTION %lf", fraction);
  
  if (fraction < 0.9){
  	return false;
  }
  trajectory = std::make_shared<moveit_msgs::RobotTrajectory>(c_trajectory);
  return true;
}

/*	--------------	*/
/*	--- STATE ----	*/

state_t FSM::start()
{
	human_baxter_collaboration::BaxterGripperOpen grip_msg;
	
	if (block != nullptr){
		grip_msg.open_gripper = true;
		gripper_pub->publish(grip_msg);
		ros::Duration(1.0).sleep();
		setBlockGrasped(block->getName(), false);	
	}
	block = nullptr;
	
	return REACH;
	
}

state_t FSM::reachBlock()
{
	human_baxter_collaboration::BaxterGripperOpen grip_msg;
	grip_msg.open_gripper = false;
	gripper_pub->publish(grip_msg);
	if (getClosestBlock(current_pose) < 0){
	// if call returns negative means no obj was retrieved
	// the table is shy of blue blocks
		
		return IDLE;
	}
	
	goal_pose = offsetGoal(block->getPose());	
	// plan and publish it
	
	return generatePlan(goal_pose) ? PICK : ERR;
	
	// return next_state;
}

state_t FSM::pickBlock()
{
	// open the grippers, go down towards the obj
	// NOTE: 	we assume here that the grippers have enough time to open before
	// 				the obj is reached, might have to add a delay here
	// reach obj pose + offset
	
	// When calling this foo perform the "get closest block" and, if it differs
	// from the previous one, go to REACH (looping?)
	human_baxter_collaboration::BaxterGripperOpen grip_msg;
	if (block == nullptr){	return ERR;	}
	
	setBlockGrasped(block->getName(), true);  //< technically not grasped yet,
																						//	but setting this here makes sense
	
	grip_msg.open_gripper = true;
	gripper_pub->publish(grip_msg);
			
			// go down to the block, being sure to correctly orient the eef
	goal_pose = block->getPose();
	// goal_pose.position.z -= 0.01; // try to go a little below the midpoint
	graspQuat(&(goal_pose.orientation));
	
	geometry_msgs::Pose mid_rot;
	mid_rot.position 		=	current_pose.position;
	mid_rot.orientation = goal_pose.orientation;
	
	return generateCartesian(mid_rot, goal_pose) ? RAISE : ERR;
}

state_t FSM::raiseBlock()
{
	human_baxter_collaboration::BaxterGripperOpen grip_msg;
	if (block == nullptr){
		return ERR;
	}
	grip_msg.open_gripper = false;
	gripper_pub->publish(grip_msg);
	
	ros::Duration(0.5).sleep();	//< here as well, we assume they have time to close
	
	goal_pose = offsetGoal(current_pose);	// go back up a bit
	
	geometry_msgs::Pose end_rot;
	end_rot.position 		=	goal_pose.position;
	end_rot.orientation = current_pose.orientation;
	
	return generateCartesian(end_rot, goal_pose) ? 
					(block->isBlue() ? PLACE_BLUE : REMOVE_RED) : ERR;
}

state_t FSM::placeBlue()
{
	if (block == nullptr){	return ERR;	}
	goal_pose = offsetGoal(BLOCK_DEST_);	// go back up a bit
			
	
	return generatePlan(goal_pose) ? START : ERR;	//< the gripper will open in START			
}

state_t FSM::removeRed()
{
	if (block == nullptr){
		return ERR;
	}
	sofar_hbc_01::ClosestEmptySpace ces;
	ces.request.arm = ARM;
	ces.request.eef_pose = block->getPose();
	
	if(!client_ces->call(ces)){
		return ERR;
	}
	goal_pose = offsetGoal(ces.response.empty_pose);
			
	 // plan and publish it
	return generatePlan(goal_pose) ? START : ERR;	//< the gripper will open in START	
}

state_t	FSM::rest()
{
	// open the gripper to release the obj (if any)
	human_baxter_collaboration::BaxterGripperOpen grip_msg;
	if (block != nullptr)
	{
		setBlockGrasped(block->getName(), false);
		grip_msg.open_gripper = true;
		gripper_pub->publish(grip_msg);
		ros::Duration(1.0).sleep();
		BAXTER_ATTEMPTS_ = 0;
	}
		
	if (BAXTER_ATTEMPTS_ == 0)
	{
		generatePlan(baxter_rest_pose_);
	}
	return ++BAXTER_ATTEMPTS_ <= MAX_BAXTER_ATTEMPTS_ ? START : END;
}


bool FSM::stateEvolution(){

	bool auto_evolve = true;
  
	switch (state){
		case START:
		{
  		ROS_INFO("BAXTER_FSM_%s: START", ARM.c_str());
			next_state = FSM::start();
			// auto_evolve = true; //< IT MUST evolve automatically from start to reach/err
		}
		case REACH:
		{
  		ROS_INFO("BAXTER_FSM_%s: REACH", ARM.c_str());
			
			next_state = FSM::reachBlock();
			
			if (next_state == IDLE){
				ROS_INFO("BAXTER_FSM_%s: ALL BLOCKS GRASPED FOR NOW (%d)", ARM.c_str(), BAXTER_ATTEMPTS_);
				break;
			}
			if (next_state == ERR){
				ROS_INFO("BAXTER_FSM_%s: ERROR DURING REACH", ARM.c_str());
				break;
			}
			// baxter_at_rest_ = false; //< this is incredibly ugly!
  		ROS_INFO("\t BLOCK FOUND: %s", block->getName().c_str());
			BAXTER_ATTEMPTS_ = 0;
			auto_evolve = false;
			
			break;
		}
		case PICK:
		{
			geometry_msgs::Pose prev_block_pose;
			// the block pose is saved since here below we search for
			// a possibly closer block
			prev_block_pose = block->getPose();
			if (FSM::getClosestBlock(current_pose) < 0){
				ROS_ERROR("BAXTER_FSM_%s: Error during PICK.", ARM.c_str());
				next_state = ERR;
				break;
			}
			/*	If the block we initially went for is now farther then expected
					go back into REACHing mode, setting the FSM to run autonomously
					(since it won't receive a result from the simulation, which is
					not carring out any trajectory).
				*/
			if (dist3(prev_block_pose, block->getPose()) > maxInterBlockDist){
				next_state = REACH;
				break;
			}
			
  		ROS_INFO("BAXTER_FSM_%s: PICK", ARM.c_str());
			next_state = FSM::pickBlock();
			
			if (next_state == ERR){
				ROS_ERROR("BAXTER_FSM_%s: Error during PICK.", ARM.c_str());
				break;
			}
			
			auto_evolve = false;
			break;
		}
		
		case RAISE:
		{
			// raise the eef up of few cm
			// next_state = obj.isBlue() ? PLACE_BLUE : REMOVE_RED
			
  		ROS_INFO("BAXTER_FSM_%s: RAISE", ARM.c_str());
			next_state = FSM::raiseBlock();
			
			if (next_state == ERR){
				ROS_ERROR("BAXTER_FSM_%s: Error during PICK.", ARM.c_str());
				break;
			}
			auto_evolve = false;
			break;
		}
		case PLACE_BLUE:
		{
			
  		ROS_INFO("BAXTER_FSM_%s: PLACE_BLUE", ARM.c_str());
  		next_state = FSM::placeBlue();
  		
  		// implement a try-catch and throw exception from when block == nullptr
			if (next_state == ERR){
				ROS_ERROR("BAXTER_FSM_%s: Error during PLACE_BLUE", ARM.c_str());
				break;
			}
			
  		ROS_INFO("\t BLOCK PLACED: %s", block->getName().c_str());
			auto_evolve = false;
			break;
		}
		case REMOVE_RED:
		{
			// retrieve empty position
			// reach the position + offset
			
  		ROS_INFO("BAXTER_FSM_%s: REMOVE_RED", ARM.c_str());
			next_state = FSM::removeRed();

			if (next_state == ERR){
				ROS_ERROR("BAXTER_FSM_%s: Error during REMOVE_RED", ARM.c_str());
				break;
			}
			 // plan and publish it
  		ROS_INFO("\t BLOCK MOVED: %s", block->getName().c_str());
			auto_evolve = false;
			break;
		}
		case END:
		{
			// Do nothing for now
			// Should it (indirectly) terminate the node?
  		ROS_INFO("BAXTER_FSM_%s: END", ARM.c_str());
  		next_state = END;
			break;
		}
		case ERR:
		{
			ROS_INFO("BAXTER_FSM_%s: ERR", ARM.c_str());
			// in case of error perform in the same way as for IDLE
		}
		case IDLE:
		{
			ROS_WARN("Going to rest state (%d)", BAXTER_ATTEMPTS_);
			next_state = FSM::rest();
			// auto_evolve = true;	//< already true
			break;
		}
	}
	
  return auto_evolve;
}


void FSM::clock_Cllbck(const ros::TimerEvent&)
{
	if (!evolve){	return;	}
	
	evolve = false;
	human_baxter_collaboration::BaxterTrajectory bxtr_traj;
	current_pose = move_group_interface->getCurrentPose().pose;
	// set it to false to avoid multiple query
	// whether to auto_evolve or not
	state = next_state;
	evolve = FSM::stateEvolution();
	// update the current status
	
	if (trajectory != nullptr)
	{
		ROS_DEBUG("BAXTER_FSM_%s: SENDING TRAJECTORY", ARM.c_str());
		bxtr_traj.arm = ARM;
		bxtr_traj.trajectory.push_back(*trajectory);
		traj_pub->publish(bxtr_traj);	// publish the plan trajectory
		trajectory = nullptr;
	}
}

void FSM::trajectoryResult(const human_baxter_collaboration::BaxterResultTrajectory::ConstPtr& msg)
{
	if (msg->arm != ARM){ return; } // not meant for this node!
	
	ROS_DEBUG("BAXTER_FSM_%s: RECEIVED RESULT", ARM.c_str());
	if (!msg->success){	
		next_state = state;
		ROS_WARN("Current trajectory interrupted, replanning.");
	}	//< rollback
	evolve = true;
}

// NOTE: theoretically, a lot of race conditions might appear with evolve, state, next_state
// Nevertheless, due to how the state progression is designed, they should in the concrete case
// never happen, since the trajectoryResult is expected to arrive only when evolve = false, and thus
// the states are not mutating
// In case, somehow, things go awry anyway, add a mutex to the call to FSM_evolve and state = next_state,
// as well as in the trajectoryResult state rollback.


