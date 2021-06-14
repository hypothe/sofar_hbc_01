/* Author: Marco G. Fedozzi */

#ifndef LINK_H
#define LINK_H

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "fcl/fcl.h"
#include <memory>

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "fcl/narrowphase/collision_object.h"

#include "sofar_hbc_01/utils.h"

class Link
{

	private:
	
		std::string name;
		geometry_msgs::Pose pose;
		geometry_msgs::Point start_point;
		double length;
		double radius;
		std::shared_ptr<Link> next;
		std::shared_ptr<fcl::Cylinderf> cyl;
		std::shared_ptr<fcl::CollisionObjectf> coll;
	
	public:
	
		Link(std::string name, double radius, double length);
		
		void setPose(geometry_msgs::Pose p);
		void setNext(std::shared_ptr<Link> next);
		
		void updateCollisionObject(); //< call this only after updating the pose
		double getRadius();
		double getLength();
		std::string getName();
		geometry_msgs::Pose getPose();
		std::shared_ptr<Link> getNext();
		std::shared_ptr<fcl::CollisionObjectf> getCollisionObject();
		
};

#endif
