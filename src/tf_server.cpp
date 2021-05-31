/* Author: Georgii A. Kurshakov */

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

bool isBlock(std::string frame_id)
{
    if (frame_id.length() == 1) //if child_frame_id is a char
        return true;
    if (frame_id == "Bluebox")
        return true;
    if (frame_id == "Redbox")
        return true;
    if (frame_id == "MiddlePlacement")
        return true;
    if (frame_id == "MiddlePlacementN")
        return true;
    return false;   //if frame_id is not "world"
}

bool blocks_callback(sofar_hbc_01::BlocksPoses::Request &req, sofar_hbc_01::BlocksPoses::Response &res)
{
    res.blocks_poses = block_frames;
    return true;
}

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

void unity_callback(const human_baxter_collaboration::UnityTf::ConstPtr& msg_in)
{
    std::vector<geometry_msgs::PoseStamped> frames = msg_in->frames;
    std::vector<geometry_msgs::PoseStamped> human_frames;
    
    block_frames.clear();
    
    for (auto frame : frames)
    {
        if (isBlock(frame.header.frame_id))
        {
            block_frames.push_back(frame);
        }
        else
        {
            human_frames.push_back(frame);
        }
    }
    
    human_baxter_collaboration::UnityTf msg_out;
    msg_out.frames = human_frames;
    human_pub.publish(msg_out);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_server");
    ros::NodeHandle n;
    
    ros::Subscriber tf_sub = n.subscribe("/tf", 1000, tf_callback);
    ros::Subscriber unity_sub = n.subscribe("/unity_tf", 1000, unity_callback);
    
    ros::ServiceServer service = n.advertiseService("/tf/blocks", blocks_callback);
    
    human_pub = n.advertise<human_baxter_collaboration::UnityTf> ("/tf/human", 1);
    baxter_pub = n.advertise<tf2_msgs::TFMessage> ("/tf/baxter", 1);
    
    ros::spin();
    return 0;
}
