/* Author: Marco G. Fedozzi */

#include "sofar_hbc_01/utils.h"

void waitForServices (std::vector<std::shared_ptr<ros::ServiceClient> > clients){
	bool ready=false;
	
	while(!ready){
		ready = true;
		for(auto client : clients){
			if (!client->exists()){
				ROS_WARN("Service %s not found.", client->getService().c_str());
				client->waitForExistence(ros::Duration(1));
				ready = false;
			}
		}
	}
	return;
}


double dist2(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2){
	return std::sqrt(	std::pow(pose2.position.x - pose1.position.x, 2) + 
										std::pow(pose2.position.y - pose1.position.y, 2)	);
	
}
double dist3(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2){
	return std::sqrt(	std::pow(pose2.position.x - pose1.position.x, 2) + 
										std::pow(pose2.position.y - pose1.position.y, 2) + 
										std::pow(pose2.position.z - pose1.position.z, 2)	);
	
}

