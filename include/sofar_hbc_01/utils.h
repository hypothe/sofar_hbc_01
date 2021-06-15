/****************************************//**
* \file utils.h
* \brief Utility functions definition
* \author Marco Gabriele Fedozzi (5083365@studenti.unige.it)
* \version 1.0
* \date 14/06/2021
*
* \details
*
* Description:
*
* This script defines some utility functions
*
********************************************/

#ifndef UTIL_H
#define UTIL_H

#include "ros/ros.h"
#include <memory>

#include "geometry_msgs/Pose.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include "fcl/fcl.h"


/****************************************//**
* Sig function
*
*	\param val (<T>): to retrieve the sign of
*
********************************************/
template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

/****************************************//**
* Wait for services to go up
*
* Loops on the vector of clients passed
* until all the services required by those
* are detected.
*
*	\param clients (std::vector<std::shared_ptr<ros::ServiceClient> >)
*
********************************************/
void waitForServices (std::vector<std::shared_ptr<ros::ServiceClient> > clients);

/****************************************//**
* 2D (XY) Euclidean distance between two poses
*
*	\param pose1 (geometry_msgs::Pose)
*	\param pose2 (geometry_msgs::Pose)
*
*	\retval dist2 (double): the planar XY
*													Euclidean distance
*
********************************************/
double dist2(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);

/****************************************//**
* 3D (XYZ) Euclidean distance between two poses
*
*	\param pose1 (geometry_msgs::Pose)
*	\param pose2 (geometry_msgs::Pose)
*
*	\retval dist3 (double): the spatial XYZ
*													Euclidean distance
*
********************************************/
double dist3(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);

/****************************************//**
* 3D (XYZ) Euclidean distance between vectors
*
*	\param vec1 (fcl::Vector3f)
*	\param vec2 (fcl::Vector3f)
*
*	\retval dist3 (double): the spatial XYZ
*													Euclidean distance
*
********************************************/
double dist3(fcl::Vector3f vec1, fcl::Vector3f vec2);

/****************************************//**
* Generate a box-like collision object.
*
*	\param frame_id (std::string): 				the parent frame name
*	\param obst_name (std::string): 			the obstacle frame name
*	\param box_pos (std::vector<double>): the x,y,z position
*																				of the obstacle
*	\param box_pos (std::vector<double>): the x,y,z dimensions
*																				of the obstacle
*	\param inflate (std::vector<double>): an "inflation" amount
*																				to add to each dimension
*																				of the obstacle (optional)
*
*	\retval obstacle (moveit_msgs::CollisionObject): the box obstacle
*
********************************************/
moveit_msgs::CollisionObject genBoxObst(
								std::string frame_id,
								std::string obst_name,
								std::vector<double> box_pos,
								std::vector<double> box_dim,
								std::vector<double> inflate = std::vector<double>(3, 0.0));

#endif
