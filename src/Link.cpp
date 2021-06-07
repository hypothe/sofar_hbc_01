/* Author: Marco G. Fedozzi */

#include "sofar_hbc_01/Link.h"

Link::Link(std::string name, double radiu, double lengths)
	: name(name), radius(radius), length(length)
{
	cyl = std::make_shared<fcl::Cylinderf>(radius, length);
	coll = std::make_shared<fcl::CollisionObjectf>(cyl);
}


void Link::setStartPoint(geometry_msgs::Point p)
{
	start_point = p;
}

geometry_msgs::Point Link::getStartPoint()
{
	return start_point;
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
std::shared_ptr<fcl::CollisionObjectf> Link::getCollisionObject()
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
	
	return coll;
}
		
