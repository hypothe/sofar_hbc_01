#include "sofar_hbc_01/Block.h"

Block::Block(std::string name, std::string color)
	: name(name), color(color)
{
	obstructed_by = "";
	placed = false;
}

void Block::setObstruction(std::string obstructed_by_){
	obstructed_by = obstructed_by_;
}

void Block::setPose(geometry_msgs::Pose pose_){
	pose = pose_;
}
void Block::setPlaced(bool placed_){
	placed = placed_;
}
		
std::string Block::getName(){
	return name;
}		
std::string Block::getColor(){
	return color;
}
bool Block::isBlue(){
	return (color == "b" || color == "blue");
}

std::string Block::getObstructedBy(){
	return obstructed_by;
}

geometry_msgs::Pose Block::getPose(){
	return pose;
}

bool Block::getPlaced(){
	return placed;
}

