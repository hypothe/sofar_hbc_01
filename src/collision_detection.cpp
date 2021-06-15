/****************************************//**
* \file collision_detection.cpp
* \brief Node performing periodic collision detection
* \author Marco Gabriele Fedozzi (5083365@studenti.unige.it)
* \version 1.0
* \date 14/06/2021
*
* \details
*
* **ServiceServer:**<BR>
*   `/baxter/collision_detection/toggle` (sofar_hbc_01::CollisionDetectionToggle)<BR>
*
* **ServiceClient:**<BR>
*   `/baxter/collision_detection/left/result` (sofar_hbc_01::CollisionDetectionResult)<BR>
*
* **ServiceClient:**<BR>
*   `/baxter/collision_detection/right/result` (sofar_hbc_01::CollisionDetectionResult)<BR>
*
* **Publishes to:**<BR>
*		`/baxter_moveit_trajectory/stop` (human_baxter_collaboration::BaxterStopTrajectory)<BR>
*
* **Subscribes to:**<BR>
*		`/tf` (tf2_msgs::TFMessage)<BR>
*		`/tf_static` (tf2_msgs::TFMessage)<BR>
*
* Description:
*
* This node holds an inner representation
*	of the Baxter and Human arms using data from
*	urdfs (the official Baxter one and a custom
* human one) to generate cylinder links.
*	Using the Flexible Collision Library tools,
*	these cylinders are treated as CollisionObjects
*	and stored in three managers groups:
*	1.	Human arms
*	2.	Baxter left arm
*	3.	Baxter right arm
*	A periodic callback (50Hz) retrieves the data
*	of the baxter and human joints from `/tf`,
* using the positions and orientations to
* update the CollisionObjects. It then
*	performs distance checks between the three
*	managers, considering a collision as possible
*	if the distance gets lower then an arbitrary
*	threshold.
* In that case a message is published on
* `/baxter_moveit_trajectory/stop` in order
*	to make the Unity simulation (or, ideally,
*	the robot controller) stop. At the same time
*	a service request is issued to the FSM of
* the arm(s) involved in the collision, with
*	a notion of the "severity" of the collision,
* which can thus be served by different recovery
* policies.
* The collision detection can be enabled or
* disabled upon request (which might be 
*	necessary) for some recovery policies.
*
********************************************/

#include "ros/ros.h"
#include <ros/console.h>

#include <memory>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "fcl/fcl.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/distance.h"
#include "fcl/narrowphase/distance_request.h"
#include "fcl/narrowphase/distance_result.h"
#include "fcl/broadphase/default_broadphase_callbacks.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include "urdf/model.h"

#include "sofar_hbc_01/Link.h"
#include "sofar_hbc_01/utils.h"
#include "sofar_hbc_01/CollisionDetectionToggle.h"
#include "sofar_hbc_01/CollisionDetectionResult.h"
#include "human_baxter_collaboration/BaxterStopTrajectory.h"

using Real = typename fcl::constants<double>::Real;

const std::string human_urdf = std::string("human_description");
const std::string baxter_urdf = std::string("baxter_description");
const double check_period_ = 0.02;

const double bxtr_wrist_ext_len = 0.2; //< model the joints after the wrist only with one joint but longer

tf2_ros::Buffer tfBuffer;
std::shared_ptr<tf2_ros::TransformListener> tfListener;

std::map<std::string, Link> human_arms, bxtr_armL, bxtr_armR;
const float human_bxtr_dist_th_ = 0.07; //< totally arbitratry for now;
const float bxtr_bxtr_dist_th_ = 0.08; //< totally arbitrary for now;
std::unique_ptr<fcl::BroadPhaseCollisionManagerf> mngr_human, mngr_bxtrL, mngr_bxtrR;

ros::Publisher stop_pub;

bool cd_on_L = true;
bool cd_on_R = true;


ros::ServiceClient client_cd_res_L, client_cd_res_R;

void setCollisionL(bool collision)
{
	if (cd_on_L != collision){ROS_WARN("COLLISION DETECTION LEFT %s", collision ? "ENABLED" : "DISABLED");}
	
	cd_on_L = collision;
}
void setCollisionR(bool collision)
{
	if (cd_on_R != collision){ROS_WARN("COLLISION DETECTION RIGHT %s", collision ? "ENABLED" : "DISABLED");}
	
	cd_on_R = collision;
}

bool toggleCD(sofar_hbc_01::CollisionDetectionToggle::Request &req,
							sofar_hbc_01::CollisionDetectionToggle::Response &res)
{
	if (req.arm == "left")	{	setCollisionL(req.cd_on);	}
	else										{	setCollisionR(req.cd_on);	}
	return true;
}

void collisionCheck(const ros::TimerEvent&)
{
	// vvv If collision detection is momentarily disabled ignore this callback vvv
	if (!cd_on_R && !cd_on_L)	{return;}
	
	auto start = std::chrono::high_resolution_clock::now();
	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::Pose pose;
	// is this temp vector creation too slow?
	for (auto link_map : std::vector<std::map<std::string, Link> >{human_arms, bxtr_armL, bxtr_armR})
	{
		for (auto link : link_map)
		{
		  try{
		    transformStamped = tfBuffer.lookupTransform("world", link.first, 
		                             ros::Time(0));
		    pose.position.x = transformStamped.transform.translation.x;
		    pose.position.y = transformStamped.transform.translation.y;
		    pose.position.z = transformStamped.transform.translation.z;
		    pose.orientation = transformStamped.transform.rotation;
		    		    
		    link.second.setPose(pose);
		    link.second.updateCollisionObject();
		  }
		  catch (tf2::TransformException &ex) {
		    ROS_WARN("%s",ex.what());
		    return; // ignore this check if info are not up-to-date
		  }
		}
	}
	
	
	mngr_human->update();
	mngr_bxtrL->update();
	mngr_bxtrR->update();
	
	fcl::DefaultDistanceData<float> distance_data_hl, distance_data_hr,  distance_data_rl;
	/*	DEBUG	
	distance_data_hl.request.enable_nearest_points = true;
	distance_data_hr.request.enable_nearest_points = true;
	distance_data_rl.request.enable_nearest_points = true;
		\DEBUG	*/
	
	if (cd_on_L)
	{
		mngr_human->distance(mngr_bxtrL.get(), &distance_data_hl, fcl::DefaultDistanceFunction);
		if (distance_data_hl.result.min_distance < human_bxtr_dist_th_)
		{
			ROS_ERROR("HUMAN - BAXTER_LEFT_ARM COLLISION INCOMING %lf", distance_data_hl.result.min_distance);
			/* DEBUG
			fcl::Vector3f p1, p2;
			p1 = distance_data_hl.result.nearest_points[0];
			p2 = distance_data_hl.result.nearest_points[1];
			ROS_DEBUG("TWO CLOSEST POINTS [%lf %lf  %lf], [%lf %lf  %lf]", p1[0], p1[1], p1[2],  p2[0], p2[1], p2[2]);
				\DEBUG */
				
			// vvv avoid bursts of collisions vvv
			setCollisionL(false);
			// LEFT arm too close to human, interrupt current trajectory for that ARM
			human_baxter_collaboration::BaxterStopTrajectory stop_msg;
			stop_msg.arm = "left";
			stop_pub.publish(stop_msg);
			
			sofar_hbc_01::CollisionDetectionResult cdres;
			cdres.request.arm = "left";
			cdres.request.severity = "LOW";
			client_cd_res_L.call(cdres);
			
			setCollisionL(cdres.response.cd_on);
		}
	}
	if (cd_on_R)
		{
		mngr_human->distance(mngr_bxtrR.get(), &distance_data_hr, fcl::DefaultDistanceFunction);
		if (cd_on_R && distance_data_hr.result.min_distance < human_bxtr_dist_th_)
		{
			ROS_ERROR("HUMAN - BAXTER_RIGHT_ARM COLLISION INCOMING %lf", distance_data_hr.result.min_distance);
			
			
			// vvv avoid bursts of collisions vvv
			setCollisionR(false);
			// RIGHT arm too close to human, interrupt current trajectory for that ARM
			human_baxter_collaboration::BaxterStopTrajectory stop_msg;
			stop_msg.arm = "right";
			stop_pub.publish(stop_msg);
			
			sofar_hbc_01::CollisionDetectionResult cdres;
			cdres.request.arm = "right";
			cdres.request.severity = "LOW";
			client_cd_res_R.call(cdres);
			setCollisionR(cdres.response.cd_on);
		}
	}
	
	if (cd_on_L && cd_on_R)
	{
		mngr_bxtrL->distance(mngr_bxtrR.get(), &distance_data_rl, fcl::DefaultDistanceFunction);
		if (distance_data_rl.result.min_distance < bxtr_bxtr_dist_th_)
		{
			ROS_ERROR("BAXTER_LEFT_ARM - BAXTER_RIGHT_ARM COLLISION INCOMING %lf", distance_data_rl.result.min_distance);
			
			// LEFT arm too close to RIGHT arm, interrupt current trajectory for both
			// They could temporarily lock if they try to replan at the same time.
			// Not a deadlock though, since after N-attempts of failed plans they will resort too
			// going back to the rest pose and re-start planning from there.
			
			// vvv avoid bursts of collisions vvv
			setCollisionL(false);
			setCollisionR(false);
			/////////////////////////////////////
			human_baxter_collaboration::BaxterStopTrajectory stop_msg;
			stop_msg.arm = "left";
			stop_pub.publish(stop_msg);
			stop_msg.arm = "right";
			stop_pub.publish(stop_msg);
			/*	Directly inform FSM of collision,	expresing how severe
					of a collision it was.
					Modify the inner flags on whether to keep checking for
					collisions or not depending on the response.
			*/
			sofar_hbc_01::CollisionDetectionResult cdres;
			
			cdres.request.arm = "left";
			cdres.request.severity = "HIGH";
			client_cd_res_L.call(cdres);
			setCollisionL(cdres.response.cd_on);
			
			cdres.request.arm = "right";
			cdres.request.severity = "HIGH";
			client_cd_res_R.call(cdres);
			setCollisionR(cdres.response.cd_on);
		}
	}
	
	auto stop = std::chrono::high_resolution_clock::now();
	
	std::chrono::duration<double> elapsed = stop - start;
	ROS_DEBUG("It took %lf seconds.", elapsed.count());
}


void fillMap(	std::vector<std::string> v_link_names, 
							std::map<std::string, Link>& map_arm,
							const std::string model_name)
{
  urdf::Model model;
  
  if (!model.initParam(model_name)){ROS_ERROR("Error while loading %s", model_name.c_str()); ros::shutdown();}
  double len;
  for (auto link_name : v_link_names)
  {
  	try
  	{
			std::shared_ptr<const urdf::Link> u_link = model.getLink(link_name);
			std::shared_ptr<urdf::Cylinder> cyl = std::dynamic_pointer_cast<urdf::Cylinder>((model.getLink(link_name))->collision->geometry);
			
			len  = cyl->length;
			if (link_name == "left_wrist" || link_name == "right_wrist"){ len =  bxtr_wrist_ext_len; }
		 	map_arm.insert(std::pair<std::string, Link>(link_name, Link(link_name, cyl->radius, len)));
		}
		catch (const std::exception& e)
		{
			ROS_ERROR("Error while accessing %s: %s", link_name.c_str(), e.what());
		}
		
  }
}


int main(int argc, char **argv)
{
		if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
			 ros::console::notifyLoggerLevelsChanged();
		}
    ros::init(argc, argv, "collision_detection");
    ros::NodeHandle node_handle;
    
    client_cd_res_L = node_handle.serviceClient<sofar_hbc_01::CollisionDetectionResult>("/baxter/collision_detection/left/result");
    client_cd_res_R = node_handle.serviceClient<sofar_hbc_01::CollisionDetectionResult>("/baxter/collision_detection/right/result");
    
		ros::ServiceServer toggle_cd_srv = node_handle.advertiseService("/baxter/collision_detection/toggle", toggleCD);
    
    stop_pub = node_handle.advertise<human_baxter_collaboration::BaxterStopTrajectory>
  							("baxter_moveit_trajectory/stop", 1000);
    
    tfListener = std::make_shared<tf2_ros::TransformListener>(tfBuffer);
    
    std::vector<std::string> human_armL_names = {"hand_l", "lowerarm_l", "upperarm_l"};
    std::vector<std::string> human_armR_names = {"hand_r", "lowerarm_r", "upperarm_r"};
    
    std::vector<std::string> baxter_armL_names =	{	"left_wrist", "left_lower_forearm",
    																							"left_upper_forearm", "left_lower_elbow",
    																							"left_upper_elbow"};  // in the end add manually length
    std::vector<std::string> baxter_armR_names = {	"right_wrist", "right_lower_forearm",
    																							"right_upper_forearm", "right_lower_elbow",
    																							"right_upper_elbow"};  // in the end add manually length
    
    
    fillMap(human_armL_names, human_arms, human_urdf);
    fillMap(human_armR_names, human_arms, human_urdf);
    
    fillMap(baxter_armL_names, bxtr_armL, baxter_urdf);
    fillMap(baxter_armR_names, bxtr_armR, baxter_urdf);

    mngr_human = std::make_unique<fcl::DynamicAABBTreeCollisionManagerf>();
    
		mngr_bxtrL = std::make_unique<fcl::DynamicAABBTreeCollisionManagerf>();
    mngr_bxtrR = std::make_unique<fcl::DynamicAABBTreeCollisionManagerf>();
    								
    std::vector<fcl::CollisionObjectf*> vec_human, vec_bxtrL, vec_bxtrR;
    
    for (auto m_link : human_arms)
    {
    	vec_human.push_back(m_link.second.getCollisionObject().get());
    }
    mngr_human->registerObjects(vec_human);
    
    for (auto m_link : bxtr_armL)
    {
    	vec_bxtrL.push_back(m_link.second.getCollisionObject().get());
    }
    mngr_bxtrL->registerObjects(vec_bxtrL);
    
    for (auto m_link : bxtr_armR)
    {
    	vec_bxtrR.push_back(m_link.second.getCollisionObject().get());
    }
    mngr_bxtrR->registerObjects(vec_bxtrR);
    
    mngr_human->setup();
    mngr_bxtrL->setup();
    mngr_bxtrR->setup();
    
    ros::Timer coll_det_timer = node_handle.createTimer(ros::Duration(check_period_), collisionCheck);
    
    ros::spin();
    return 0;
}


