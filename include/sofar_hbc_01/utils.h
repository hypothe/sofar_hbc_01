#ifndef UTIL_H
#define UTIL_H

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

void waitForServices (std::vector<std::shared_ptr<ros::ServiceClient> > clients);

double dist2(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);
double dist3(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);

#endif
