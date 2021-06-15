/****************************************//**
* \file Block.h
* \brief Class definition of the Block item
* \author Marco Gabriele Fedozzi (5083365@studenti.unige.it)
* \version 1.0
* \date 05/06/2021
*
********************************************/
#ifndef BLOCK_H
#define BLOCK_H

#include "geometry_msgs/Pose.h"

/****************************************//**
* This class stores a representation of one
*	of the graspable blocks the EEf can interact
*	with.
*
********************************************/
class Block
{
	public:
	
		/****************************************//**
		* Block constructor
		*
		* Initialize the block as not placed and
		* unobstructed.
		*
		*	\param name_ (std::string)
		*	\param color_ (std::string)
		*
		********************************************/
		Block(std::string name_, std::string color_);
		
		/****************************************//**
		* Set the name of the block obstructing this
		* one
		*
		*	\param obstructed_by_ (std::string)
		*
		********************************************/
		void setObstruction(std::string obstructed_by_);
		
		/****************************************//**
		* Set the pose of this block.
		*
		*	\param pose_ (geometry_msgs::Pose)
		*
		********************************************/
		void setPose(geometry_msgs::Pose pose_);
		
		/****************************************//**
		* Set whether this block is already placed
		* at its destination.
		*
		*	\param placed_ (bool)
		*
		********************************************/
		void setPlaced(bool placed_);
		
		/****************************************//**
		* Get this block name.
		*
		*	\retval name (std::string)
		*
		********************************************/
		std::string getName();
		
		/****************************************//**
		* Get this block color.
		*
		*	\retval color (std::string)
		*
		********************************************/
		std::string getColor();
		
		/****************************************//**
		* Whether this block is blue.
		*
		*	\retval blue (bool)
		*
		********************************************/
		bool isBlue();
		
		/****************************************//**
		* Get the name of the block obstructing
		* this one (if any).
		*
		*	\retval obstructed_by (std::string)
		*
		********************************************/
		std::string getObstructedBy();
		
		/****************************************//**
		* Get the pose of this block.
		*
		*	\retval pose (geometry_msgs::Pose)
		*
		********************************************/
		geometry_msgs::Pose getPose();
		
		/****************************************//**
		* Get whether this block is already placed
		* at its destination.
		*
		*	\retval placed (bool)
		*
		********************************************/
		bool getPlaced();

	private:
	
		bool placed;
		std::string name;
		std::string color;
		std::string obstructed_by;
		geometry_msgs::Pose pose;
};

#endif
