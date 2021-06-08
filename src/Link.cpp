/* Author: Marco G. Fedozzi */

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
/*
void Link::initCollision(double radius)
{
	geometry_msgs::Pose next_start;
	if (next_start == nullptr)
	{
		ROS_ERROR("Uninitialized 'next' field for %s", this.name.c_str());
		return;
	}
	
	if (this.length <= 0){
		next_start = next->getStartPoint();
		length = dist3(start, next_start);
	}
	
}
*/
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
/*
void Link::updateCollisionObject()
{
	fcl::Transform3f t = fcl::Transform3f::Identity();
	
	geometry_msgs::Point next_start = next->getStartPoint();
	pose.position.x = (start_point.x + next_start.x)/2.0;
	pose.position.y = (start_point.y + next_start.y)/2.0;
	pose.position.z = (start_point.z + next_start.z)/2.0;
	
	t.translate(fcl::Vector3f(pose.position.x, pose.position.y, pose.position.z));
	fcl::Quaternionf q;
	q = q.FromTwoVectors(	fcl::Vector3f(0,0,1),
												fcl::Vector3f(next_start.x, next_start.y, next_start.z));
												
	pose.orientation.x = q.x();
	pose.orientation.y = q.y();
	pose.orientation.z = q.z();
	pose.orientation.w = q.w();
	
	t.rotate(q);
	
	coll->setTransform(t);
}*/
void Link::updateCollisionObject()
{
	fcl::Transform3f t = fcl::Transform3f::Identity();
	// Get curernt link heading
	fcl::Quaternionf q(	pose.orientation.w, pose.orientation.x,
											pose.orientation.y, pose.orientation.z);
	
	fcl::Vector3f vec_mid_dist(0.0,0.0,length/2.0), vec_start_point(pose.position.x, pose.position.y ,pose.position.z);
	// get the link middle point by adding the semi-length of the link oriented
	//fcl::Vector3f vec_mid_point = q*vec_mid_dist + vec_start_point;
	fcl::Vector3f vec_mid_point = q*vec_mid_dist + vec_start_point;
	ROS_DEBUG("MID_POINT X:%lf Y:%lf Z:%lf", vec_mid_point[0], vec_mid_point[1], vec_mid_point[2]);
	
	t.translate(vec_mid_point);
	t.rotate(q);
	
	coll->setTransform(t);
}
		

std::shared_ptr<fcl::CollisionObjectf> Link::getCollisionObject()
{
	return coll;
}
