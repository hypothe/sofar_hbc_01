

#include "ros/ros.h"
#include <time.h>

#include "human_baxter_collaboration/BaxterResultTrajectory.h"
#include "iostream"

int main(int argc, char **argv){

	ros::init(argc, argv, "baxter_start_moving");
	ros::NodeHandle node_handle;
	ros::Publisher pub;
	srand(time(NULL));

  pub = node_handle.advertise<human_baxter_collaboration::BaxterResultTrajectory>("/baxter_moveit_trajectory/result", 1000);
	human_baxter_collaboration::BaxterResultTrajectory msg;
	
	ros::Duration(5.0).sleep();
	
	std::string side[] = {"left", "right"};
	
	///////////////////
	msg.success = true;
	
	int s = rand()%2;
	
	msg.arm = side[s];
	pub.publish(msg);
	
	double ww = double(rand())/double(RAND_MAX) * 2.0;
	
	msg.arm = side[(s+1)%2];
	pub.publish(msg);
	///////////////////
	ros::Duration(1.0).sleep();
	
	ros::shutdown();

	return 0;
}
