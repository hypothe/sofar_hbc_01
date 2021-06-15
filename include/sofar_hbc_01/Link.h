/****************************************//**
* \file Link.h
* \brief Class definition of the Link item
* \author Marco Gabriele Fedozzi (5083365@studenti.unige.it)
* \version 1.0
* \date 14/06/2021
*
********************************************/
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

/****************************************//**
* This class stores a representation of one
*	link used for collision detection purposes,
* by using FCL primitives and tools.
*
********************************************/
class Link
{

	public:
	
		/****************************************//**
		* Link constructor
		*
		* Generate a cylinder from the parameters.
		*
		*	\param name (std::string)
		*	\param radius (double)
		*	\param length (double)
		*
		********************************************/
		Link(std::string name, double radius, double length);
		
		/****************************************//**
		* Set the pose of this link.
		*
		*	\param pose (geometry_msgs::Pose)
		*
		********************************************/
		void setPose(geometry_msgs::Pose p);
		
		/****************************************//**
		* Set the next link in the chain.
		*
		* Currently unused.
		*
		*	\param next (std::shared_ptr<Link>)
		*
		********************************************/
		void setNext(std::shared_ptr<Link> next);
		
		/****************************************//**
		* Update the collision object associated
		*	with the link
		*
		*	The current link informations, such as
		*	position and orientation, are used to
		* modify the collision object. For such
		* reason, this function should be called
		* only after setting the desired link pose,
		* ideally only before using such object.
		*
		********************************************/
		void updateCollisionObject(); //< call this only after updating the pose
		
		
		/****************************************//**
		* Get this link radius.
		*
		*	\retval radius (double)
		*
		********************************************/
		double getRadius();
		
		/****************************************//**
		* Get this link length.
		*
		*	\retval length (double)
		*
		********************************************/
		double getLength();
		
		/****************************************//**
		* Get this link name.
		*
		*	\retval name (std::string)
		*
		********************************************/
		std::string getName();
		
		/****************************************//**
		* Get this link pose.
		*
		*	\retval pose (geometry_msgs::Pose)
		*
		********************************************/
		geometry_msgs::Pose getPose();
		
		/****************************************//**
		* Get the next link in the chain.
		*
		* Currently unused.
		*
		*	\retval next (std::shared_ptr<Link>)
		*
		********************************************/
		std::shared_ptr<Link> getNext();
		
		/****************************************//**
		* Get the collision object associated with
		* this link.
		*
		*	\retval coll (std::shared_ptr<fcl::CollisionObjectf>)
		*
		********************************************/
		std::shared_ptr<fcl::CollisionObjectf> getCollisionObject();
		
	private:
	
		std::string name;
		geometry_msgs::Pose pose;
		geometry_msgs::Point start_point;
		double length;
		double radius;
		std::shared_ptr<Link> next;
		std::shared_ptr<fcl::Cylinderf> cyl;
		std::shared_ptr<fcl::CollisionObjectf> coll;
	
};

#endif
