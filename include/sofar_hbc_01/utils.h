#ifndef UTIL_H
#define UTIL_H

#include "ros/ros.h"

void waitForServices (std::vector<std::shared_ptr<ros::ServiceClient> > clients);

#endif
