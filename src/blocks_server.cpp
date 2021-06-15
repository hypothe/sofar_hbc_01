/**
 * @file blocks_server.cpp
 * @author Georgii A. Kurshakov (kurshakov98@gmail.com)
 * @brief A node reading data from /tf, /unity_tf and transferring the data to /tf/blocks service.
 * @date 2021-05-31
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "sofar_hbc_01/BlocksPoses.h"
#include "human_baxter_collaboration/UnityTf.h"

#include <vector>
#include <string>

ros::Publisher human_pub;
ros::Publisher baxter_pub;

std::vector<geometry_msgs::PoseStamped> block_frames;

/**
 *  \brief Check if the current frame is a block.
 *  
 *  \param frame_id Object frame ID.
 *  \return 1 if the object is a block, 2 if the object is a scene object (must be discarded), 0 if the object is a human link.
 *  
 */
int isBlock(std::string frame_id)
{
    if (frame_id.length() == 1) //if frame_id is a single character
        return 1;
    if (frame_id == "Bluebox")
        return 2;
    if (frame_id == "Redbox")
        return 2;
    if (frame_id == "MiddlePlacement")
        return 2;
    if (frame_id == "MiddlePlacementN")
        return 2;
    return 0;
}

/**
 *  \brief /tf/blocks service callback.
 *  
 *  \param req Service request (empty signal).
 *  \param res Service responce (vector of frames).
 *  \return True
 *  
 */
bool blocks_callback(sofar_hbc_01::BlocksPoses::Request &req, sofar_hbc_01::BlocksPoses::Response &res)
{
    res.blocks_poses = block_frames;
    return true;
}

/**
 *  \brief /tf topic callback, transferring all the data directly to /tf/baxter topic.
 *  
 *  \param msg_in Topic message.
 *  
 */
 /*
void tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg_in)
{
    std::vector<geometry_msgs::TransformStamped> frames = msg_in->transforms;
    std::vector<geometry_msgs::TransformStamped> baxter_frames;
    
    for (auto frame : frames)
    {
        baxter_frames.push_back(frame);
    }
    
    tf2_msgs::TFMessage msg_out;
    msg_out.transforms = baxter_frames;
    baxter_pub.publish(msg_out);
}
*/
/**
 *  \brief /unity_tf topic callback, sorting human and scene data.
 *  
 *  \param msg_in Topic message.
 *  
 *  Blocks data is being stored awaiting /tf/blocks service request. Human data is being transferred directly to /tf/human topic.
 */
void unity_callback(const human_baxter_collaboration::UnityTf::ConstPtr& msg_in)
{
    std::vector<geometry_msgs::PoseStamped> frames = msg_in->frames;
    // std::vector<geometry_msgs::PoseStamped> human_frames;
    
    block_frames.clear(); //the blocks vector is being cleared before writing new values
    
    for (auto frame : frames)
    {
        if (isBlock(frame.header.frame_id) == 1)
        {
            block_frames.push_back(frame);
        }
        /*else if (isBlock(frame.header.frame_id) == 0)
        {
            human_frames.push_back(frame);
        }*/
    }
    
    // human_baxter_collaboration::UnityTf msg_out;
    // msg_out.frames = human_frames;
    // human_pub.publish(msg_out);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_server");
    ros::NodeHandle n;
    
    // ros::Subscriber tf_sub = n.subscribe("/tf", 1000, tf_callback);
    ros::Subscriber unity_sub = n.subscribe("/unity_tf", 1000, unity_callback);
    
    ros::ServiceServer service = n.advertiseService("/tf/blocks", blocks_callback);
    
    // human_pub = n.advertise<human_baxter_collaboration::UnityTf> ("/tf/human", 1);
    // baxter_pub = n.advertise<tf2_msgs::TFMessage> ("/tf/baxter", 1);
    
    ros::spin();
    return 0;
}
