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

class Link
{

	private:
	
		std::string name;
		geometry_msgs::Pose pose;
		geometry_msgs::Point start_point;
		//geometry_msgs::Point mid_point;	//< contained in pose
		double length;
		double radius;
		//geometry_msgs::Quaternion orientation;	//< contained in pose
		std::shared_ptr<Link> next;
		std::shared_ptr<fcl::Cylinderf> cyl;
		std::shared_ptr<fcl::CollisionObjectf> coll;
	
	public:
	
		Link(std::string name, double radius, double length);
		
		void setPose(geometry_msgs::Pose p);
		void setNext(std::shared_ptr<Link> next);
		// void initCollision(double radius);
		
		void updateCollisionObject(); //< call this only after updating the pose
		/*
		void setOrientation(geometry_msgs::Quaternion q);
		void setmid_point(geometry_msgs::Point p);
		void setPose(geometry_msgs::Pose pose);
		*/
		double getRadius();
		double getLength();
		std::string getName();
		geometry_msgs::Pose getPose();
		std::shared_ptr<Link> getNext();
		std::shared_ptr<fcl::CollisionObjectf> getCollisionObject();
		
};

#endif
