/* Author: Marco G. Fedozzi */

#include "ros/ros.h"

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

//using fcl::Vector3;
using Real = typename fcl::constants<double>::Real;

const std::string human_urdf = std::string("human_description");
const std::string baxter_urdf = std::string("baxter_description");
const double check_period_ = 0.2;

const double bxtr_wrist_ext_len = 0.2; //< model the joints after the wrist only with one joint but longer

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);

std::map<std::string, Link> human_arms, bxtr_armL, bxtr_armR;
const float human_bxtr_dist_th_ = 0.08; //< totally arbitratry for now;
const float bxtr_bxtr_dist_th_ = 0.08; //< totally arbitratry for now;
std::unique_ptr<fcl::BroadPhaseCollisionManagerf> mngr_human, mngr_bxtrL, mngr_bxtrR;

void collisionCheck(const ros::TimerEvent&)
{
	geometry_msgs::TransformStamped transformStamped;
	geometry_msgs::Pose pose;
	// is this temp vector creation too slow?
	for (auto link_map : std::vector<std::map<std::string, Link> >{human_arms, bxtr_armL, bxtr_armR})
	{
		for (auto link : link_map)
		{
		  try{
		    transformStamped = tfBuffer.lookupTransform(link.first, "world",
		                             ros::Time(0));
		    pose.position.x = transformStamped.transform.translation.x;
		    pose.position.y = transformStamped.transform.translation.y;
		    pose.position.x = transformStamped.transform.translation.z;
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
	
	fcl::DefaultDistanceData<float> distance_data;
	
	mngr_human->distance(mngr_bxtrL.get(), &distance_data, fcl::DefaultDistanceFunction);
	if (distance_data.result.min_distance < human_bxtr_dist_th_)
	{
		ROS_ERROR("HUMAN - BAXTER_LEFT_ARM COLLISION INCOMING");
	}
	mngr_human->distance(mngr_bxtrR.get(), &distance_data, fcl::DefaultDistanceFunction);
	if (distance_data.result.min_distance < human_bxtr_dist_th_)
	{
		ROS_ERROR("HUMAN - BAXTER_RIGHT_ARM COLLISION INCOMING");
	}
	mngr_bxtrL->distance(mngr_bxtrR.get(), &distance_data, fcl::DefaultDistanceFunction);
	if (distance_data.result.min_distance < bxtr_bxtr_dist_th_)
	{
		ROS_ERROR("BAXTER_LEFT_ARM - BAXTER_RIGHT_ARM COLLISION INCOMING");
	}
	
	// TODO: msg publishing on baxter_moveit_trajectory/stop topic
	
}


void fillMap(	std::vector<std::string> v_link_names, 
							std::map<std::string, Link>& map_arm,
							const std::string model_name)
{
  urdf::Model model;
  
  if (!model.initParam(model_name)){ROS_ERROR("Error while loading %s", model_name.c_str());}
  double len;
  for (auto link_name : v_link_names)
  {
  	try
  	{
			std::shared_ptr<const urdf::Link> u_link = model.getLink(link_name);
			std::shared_ptr<urdf::Cylinder> cyl = std::dynamic_pointer_cast<urdf::Cylinder>((model.getLink(link_name))->collision->geometry);
			
			len  = cyl->length;
			if (link_name == "left_wrist" || link_name == "right_name"){ len =  bxtr_wrist_ext_len; }
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
    ros::init(argc, argv, "collision_detection");
    ros::NodeHandle node_handle;
    
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
    								
    // let's see if it accepts also shared ptr instead of naked ones
    // EDIT: nope, it doesn't
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
    
    // TESTS, IGNORE
    // vvvvvvvvvvvvv
		/*    
    geometry_msgs::Pose pose;
    pose.position.x = 1.0;
    pose.orientation.y = std::sqrt(2.0)/2.0;
    pose.orientation.w = std::sqrt(2.0)/2.0;

    if (human_arms.count("hand_l")>0)
    {
    	std::shared_ptr<Link> tmp_link = std::make_shared<Link>(human_arms.find("hand_l")->second);
		  tmp_link->setPose(pose);
		  tmp_link->updateCollisionObject();
		  std::shared_ptr<fcl::CollisionObjectf> coll = tmp_link->getCollisionObject();
		  fcl::Vector3f tran = coll->getTranslation();
		  fcl::Quaternionf quat = coll->getQuatRotation();
		  ROS_INFO("HAND_L x%lf y%lf z%lf xr%lf yr%lf zr%lf wr%lf", tran[0], tran[1], tran[2],
    																												quat.x(), quat.y(), quat.z(), quat.w());
    }
    for (auto coll : vec_human)
    {
		  fcl::Vector3f tran = coll->getTranslation();
		  fcl::Quaternionf quat = coll->getQuatRotation();
		  ROS_INFO("HAND_L x%lf y%lf z%lf xr%lf yr%lf zr%lf wr%lf", tran[0], tran[1], tran[2],
    																												quat.x(), quat.y(), quat.z(), quat.w());
    }
    std::vector<fcl::CollisionObjectf*> tmp_vec;
    mngr_human->getObjects(tmp_vec);
    for (auto coll : tmp_vec)
    {
		  fcl::Vector3f tran = coll->getTranslation();
		  fcl::Quaternionf quat = coll->getQuatRotation();
		  ROS_INFO("HAND_L x%lf y%lf z%lf xr%lf yr%lf zr%lf wr%lf", tran[0], tran[1], tran[2],
    																												quat.x(), quat.y(), quat.z(), quat.w());
    }
    */
    
    // Good news, it's enough to call mngr->update() to let it refit the inner tree!
    
    // TODO: create timer and bind coll_check to it
    
    ros::spin();
    return 0;
}


