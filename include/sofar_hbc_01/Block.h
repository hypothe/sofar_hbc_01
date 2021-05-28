#ifndef BLOCK_H
#define BLOCK_H

#include "geometry_msgs/Pose.h"

class Block
{

	private:
	
		bool placed;
		std::string name;
		std::string color;
		std::string obstructed_by;
		geometry_msgs::Pose pose;
	
	public:
	
		Block(std::string name_, std::string color_);
		
		void setObstruction(std::string obstructed_by_);
		void setPose(geometry_msgs::Pose pose_);
		void setPlaced(bool placed_);
		
		std::string getName();
		std::string getColor();
		bool isBlue();
		std::string getObstructedBy();
		geometry_msgs::Pose getPose();
		bool getPlaced();

};

#endif
