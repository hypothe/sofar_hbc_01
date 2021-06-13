/* Author: Marco G. Fedozzi */

#ifndef UTIL_H
#define UTIL_H

#include "ros/ros.h"
#include <memory>

#include "geometry_msgs/Pose.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

template <typename T> int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

void waitForServices (std::vector<std::shared_ptr<ros::ServiceClient> > clients);

double dist2(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);
double dist3(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);


moveit_msgs::CollisionObject genBoxObst(
								std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface,
								std::string obst_name,
								std::vector<double> box_pos,
								std::vector<double> box_dim,
								std::vector<double> inflate = std::vector<double>(3, 0.0));

#endif
