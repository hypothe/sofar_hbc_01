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
