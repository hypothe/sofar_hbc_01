/* Author: Marco G. Fedozzi */

#include "ros/ros.h"

#include <memory>
#include <chrono>

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "fcl/fcl.h"
//#include "eigen_matrix_compare.h"
#include "fcl/narrowphase/collision_object.h"
#include "fcl/narrowphase/distance.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

using fcl::Vector3;
using Real = typename fcl::constants<double>::Real;

void checkCollisions(double r_c1, double r_c2, double h_c1, double h_c2)
{

	const double pi = fcl::constants<double>::pi();
	
	auto cyl1 = std::make_shared<fcl::Cylinder<double> >(r_c1, h_c1);
	auto cyl2 = std::make_shared<fcl::Cylinder<double> >(r_c2, h_c2);
	
	fcl::Transform3<double> X_WC1  = fcl::Transform3<double>::Identity();
	fcl::Transform3<double> X_WC2  = fcl::Transform3<double>::Identity();
	
	Vector3<Real> curr_transl = X_WC2.translation();
	// fcl::Matrix3<double> mat_rot = X_WC2.rotation();
	/*
	fcl::Quaternion<double> q_rot(X_WC2.rotation());
	double roll = 0.0, pitch = pi/2.0, yaw = 0.0;
	fcl::Matrix3<double> mat = fcl::Matrix3<double>::Identity();
	mat = Eigen::AngleAxis<double>(roll, Vector3<double>::UnitX()) *
				Eigen::AngleAxis<double>(pitch, Vector3<double>::UnitY()) *
				Eigen::AngleAxis<double>(yaw, Vector3<double>::UnitZ());
	//X_WC2.rotation() << mat;
	*/
	Vector3<double> point(1, 0, 0);
	fcl::Quaternion<double> q; q = q.FromTwoVectors(Vector3<double>(0,0,1), Vector3<double>(1,1,0));
	X_WC2.rotate(q);
	
	Vector3<double> rpy = X_WC2.rotation().eulerAngles(0,1,2);
	
//	const Real sx = 0.5;	const Real sy = 0.5;	const Real sz = 0.5;
//	X_WC2.translation() << sx, sy, sz;
	
	ROS_INFO("Current translation2 %lf, %lf, %lf", curr_transl[0],curr_transl[1],curr_transl[2]);
	ROS_INFO("Current rotation2 %lf, %lf, %lf", rpy[0]*180.0/pi, rpy[1]*180.0/pi, rpy[2]*180.0/pi);
	
	fcl::CollisionObject<double> cyl1_co(cyl1, X_WC1);
	fcl::CollisionObject<double> cyl2_co(cyl2, X_WC2);
	
	fcl::CollisionRequest<double> req;
	fcl::CollisionResult<double> res;
	
	fcl::DistanceRequest<double> d_req;
	fcl::DistanceResult<double> d_res;
	auto start = std::chrono::high_resolution_clock::now();
	int j = 0;
	
	for (double offset=-5.0; offset<=5.0; offset+=0.001)
	{
		const double th = offset * pi;
		X_WC2.translation() << curr_transl[0]+(!(j%3))*cos(th), curr_transl[1]+(!((j+1)%3))*sin(th), 
														curr_transl[2]+ (!((j+2)%3))*(h_c1/2.0 + h_c2/2.0 + offset);
		cyl2_co.setTransform(X_WC2);
	
		fcl::collide(&cyl1_co, &cyl2_co, req, res);
		fcl::distance(&cyl1_co, &cyl2_co, d_req, d_res);
		ROS_DEBUG("OFFSET: %lf", offset);
		ROS_DEBUG("Collision result: %d", res.isCollision());
		ROS_DEBUG("Distance result: %lf", d_res.min_distance);
		
		res.clear();	d_res.clear();
		j++;
	}
	auto stop = std::chrono::high_resolution_clock::now();
	
	std::chrono::duration<double> elapsed = stop - start;
	ROS_INFO("It took %lf seconds to do %d checks.", elapsed.count(), j);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_detection");
    ros::NodeHandle nonde_handle;
    
    checkCollisions(0.1, 0.1, 0.3, 0.3);
    
    ros::spin();
    return 0;
}


