#ifndef _R2_GAZE_CONTROLLER_HPP_
#define _R2_GAZE_CONTROLLER_HPP_

// My neck inverse_kinematics
#include "r2_gaze_ik.hpp"

// Boost
#include <boost/scoped_ptr.hpp>

// Standard libraries
#include <string>

// ROS core
#include <ros/ros.h>
#include <ros/node_handle.h>

// ROS messages
#include <message_filters/subscriber.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>

// Other ROS packages
#include <tf/message_filter.h>
#include <tf/transform_listener.h>

namespace r2_gaze_controller
{

class R2GazeController
{
public:
    R2GazeController(const ros::NodeHandle& handle, const std::string& rootName, const std::string& tipName);
    ~R2GazeController();

    bool initialize();
        
private:
    KDL::Segment createVirtualSegment();
    double getWeight(const std::string& paramName);
    Eigen::MatrixXd getWeightMatrix();
    void setupRosTopics(const std::string& lookAtTopicName, const std::string& neckJointCmdTopicName);
    void serializeToMsg(const KDL::JntArray& q, sensor_msgs::JointState& cmdMsg);
    void transformToRootFrame(const geometry_msgs::PoseStamped::ConstPtr& pose_msg, KDL::Frame& frame);
    void publishNeckCmd(const KDL::JntArray& q_cmd);
    void look_at_cb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    
    // ROS comm
    ros::NodeHandle nodeHandle;
    ros::Subscriber jointStateSubscriber;
    ros::Publisher neckCmdPublisher;
    message_filters::Subscriber<geometry_msgs::PoseStamped> lookAtSubscriber;
    boost::scoped_ptr<tf::MessageFilter<geometry_msgs::PoseStamped> > lookAtFilter;
    tf::TransformListener transformListener;
    
    // Inverse kinematics solver
    boost::scoped_ptr<r2_gaze_controller::R2GazeIK> ikPtr;
    
    // Tip and root names of the chain to create
    std::string rootName, tipName;

    // Temporaries to speed up computation
    KDL::JntArray q_init, q_cmd;
    KDL::Frame F_desired;
    sensor_msgs::JointState jntCmdMsg;
};

}

#endif
