/****************************************//**
* \file Link.cpp
* \brief Class implementation of the Link item
* \author Marco Gabriele Fedozzi (5083365@studenti.unige.it)
* \version 1.0
* \date 14/06/2021
*
* \details
*
* Description:
*
* This script implements the Link class
*	defined at Link.h
*
********************************************/

#include "sofar_hbc_01/Link.h"
#include "ros/ros.h"

Link::Link(std::string name, double radius, double length)// next=nullptr)
	: name(name), radius(radius), length(length)
{
	cyl = std::make_shared<fcl::Cylinderf>(radius, length);
	coll = std::make_shared<fcl::CollisionObjectf>(cyl);
}


void Link::setPose(geometry_msgs::Pose pose)
{
	this->pose = pose;
}

void Link::setNext(std::shared_ptr<Link> next)
{
	this->next = next;
}

double Link::getRadius()
{
	return radius;
}
double Link::getLength()
{
	return length;
}
geometry_msgs::Pose Link::getPose()
{
	return pose;
}
		
std::string Link::getName()
{
	return name;
}
std::shared_ptr<Link> Link::getNext()
{
	return next;
}
// WARN: no check on time coherency done at this level!
// might switch to *Stamped and do a try-except for improved
// robustness

void Link::updateCollisionObject()
{
	fcl::Transform3f t = fcl::Transform3f::Identity();
	
	// Get curernt link heading
	fcl::Quaternionf q(	pose.orientation.w, pose.orientation.x,
											pose.orientation.y, pose.orientation.z);
	
	fcl::Vector3f vec_mid_dist(0.0,0.0,length/2.0), vec_start_point(pose.position.x,
	 																																pose.position.y,
	 																																pose.position.z);
	// get the link middle point by adding the semi-length of the link oriented
	fcl::Vector3f vec_mid_point = q*vec_mid_dist + vec_start_point;
	
	t.translate(vec_mid_point);
	t.rotate(q);

	coll->setIdentityTransform();
	coll->setTransform(t);
	// vvv This cast down here works, somehow. vvv
	fcl::Vector3f tran = coll->getTranslation();
	ROS_DEBUG("%s: START_POINT X:%lf Y:%lf Z:%lf", name.c_str(),  tran[0], tran[1], tran[2]);
	fcl::Quaternionf quat = coll->getQuatRotation();
	ROS_DEBUG("%s: QUAT X:%lf Y:%lf Z:%lf W:%lf", name.c_str(),  quat.x(), quat.y(), q.z(), q.w());
}
		

std::shared_ptr<fcl::CollisionObjectf> Link::getCollisionObject()
{
	return coll;
}
