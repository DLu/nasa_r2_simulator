#include "r2_gaze_controller.hpp"

// Boost
#include <boost/thread/mutex.hpp>

// KDL
#include <kdl/tree.hpp>

// Ros
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>

namespace r2_gaze_controller
{

int GetIndexFromName(const std::vector<std::string>& nameArray, const std::string& name)
{
    for (unsigned int n=0; n < nameArray.size(); n++)
    {
        if (name == nameArray[n]) return n;
    }    
    return -1;
}

void print_pose(const KDL::Frame& frame, const std::string& name)
{
	double r,p,y;
	frame.M.GetRPY(r, p, y);
	ROS_INFO("x_%s: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", name.c_str(), frame.p(0), frame.p(1), frame.p(2), r, p, y);
}

R2GazeController::R2GazeController(const ros::NodeHandle& handle, const std::string& rootName, const std::string& tipName) :
	nodeHandle(handle),
	ikPtr(NULL),
	rootName(rootName),
	tipName(tipName),
	q_init(4),
	q_cmd(4)
{
    jntCmdMsg.name.push_back("r2/neck/joint0");
    jntCmdMsg.name.push_back("r2/neck/joint1");
    jntCmdMsg.name.push_back("r2/neck/joint2");
	jntCmdMsg.position.resize(3);

	q_init.data = Eigen::VectorXd::Zero(4);
	q_cmd.data = Eigen::VectorXd::Zero(4);
}

R2GazeController::~R2GazeController()
{
    jointStateSubscriber.shutdown();
}

bool R2GazeController::initialize()
{
    // Create a KDL tree from the robot_description parameter
	// TODO: Make robot_description a parameter
    KDL::Tree tree;
    if (!kdl_parser::treeFromParam("robot_description", tree))
    {
        ROS_ERROR("Failed to construct kdl tree from the robot_description");
        return false;
    }
   
    // Extract the 'neck' chain from the tree and then add our virtual segment
    KDL::Chain chain;
    tree.getChain(rootName, tipName, chain);
    chain.addSegment(createVirtualSegment());
    
    // With a augmented chain created, lets create a kinematics solver
    ikPtr.reset(new R2GazeIK(chain));
    ikPtr->setWeightMatrix(getWeightMatrix());
    
    // And then setup our ROS topics
    setupRosTopics("/r2_controller/gaze/pose_command", "/r2_controller/neck/joint_command");

	return true;
}

KDL::Segment R2GazeController::createVirtualSegment()
{
	KDL::Joint virtualPrismaticJoint(KDL::Joint::TransX);
	KDL::Frame virtualFrame(KDL::Frame::Identity());
	return KDL::Segment(virtualPrismaticJoint, virtualFrame);
}

double R2GazeController::getWeight(const std::string& paramName)
{
	double weight = 1.0;
	if (nodeHandle.getParam(paramName, weight))
	{
		ROS_INFO("IN  HERE");
	}
	std::cout << paramName << ": " << weight << std::endl;
	//ROS_INFO("%s: %.1f", paramName.c_str(), weight);
	return weight;
}

Eigen::MatrixXd R2GazeController::getWeightMatrix()
{
    /* Default weight matrix. A large weight means that joint will contribute
     * to the task-space solution and a small weight means the joint will
     * not be used to come up with the task-space solution. In our case we
     * want to penalize the use of the real joints in an effort to minimize
     * the robot 'chicken head' syndrome.
	 */
    Eigen::VectorXd w(4);
    w(0) = getWeight("/r2_gaze_controller/weight_j0");
    w(1) = getWeight("/r2_gaze_controller/weight_j1");
    w(2) = getWeight("/r2_gaze_controller/weight_j2");
    w(3) = getWeight("/r2_gaze_controller/weight_jv");
    return w.asDiagonal();
}

void R2GazeController::setupRosTopics(const std::string& lookAtTopicName, const std::string& neckJointCmdTopicName)
{
	// Subscribe to the joint_state topic
	//jointStateSubscriber = nodeHandle.subscribe("/joint_states", 1, &GazeController::R2GazeController::joint_state_cb, this);

	// Subscribe to a PoseStamped topic - this is our desired look-at point
	lookAtSubscriber.subscribe(nodeHandle, lookAtTopicName, 1);
	lookAtFilter.reset(new tf::MessageFilter<geometry_msgs::PoseStamped>(lookAtSubscriber, transformListener, rootName, 10, nodeHandle));
	lookAtFilter->registerCallback(boost::bind(&r2_gaze_controller::R2GazeController::look_at_cb, this, _1));

	// Publish our joint commands to the neck command topic
	neckCmdPublisher = nodeHandle.advertise<sensor_msgs::JointState>(neckJointCmdTopicName, 1);
}

void R2GazeController::serializeToMsg(const KDL::JntArray& q, sensor_msgs::JointState& cmdMsg)
{
	for (int n=0; n < cmdMsg.position.size(); n++) cmdMsg.position[n] = q(n);
}

void R2GazeController::transformToRootFrame(const geometry_msgs::PoseStamped::ConstPtr& pose_msg, KDL::Frame& frame)
{
	// Convert ROS pose message to KDL frame format
	tf::Stamped<tf::Pose> pose_stamped;
	poseStampedMsgToTF(*pose_msg, pose_stamped);

	// Convert to reference frame of root link of the controller chain
	//transformListener.waitForTransform(rootName, tipName, ros::Time::now(), ros::Duration(1.0));
	transformListener.transformPose(rootName, pose_stamped, pose_stamped);
	tf::PoseTFToKDL(pose_stamped, frame);
}

void R2GazeController::publishNeckCmd(const KDL::JntArray& q_cmd)
{
	serializeToMsg(q_cmd, jntCmdMsg);
	neckCmdPublisher.publish(jntCmdMsg);
}

void R2GazeController::look_at_cb(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    // Transform msg pose into our root frame
	transformToRootFrame(pose_msg, F_desired);

    // Compute IK solution
    int ret = ikPtr->computeSolution(q_init, F_desired, q_cmd);

    // Publish... regardless if we converged (desired point may be unreachable)
    publishNeckCmd(q_cmd);

    if (ret < 0)
    {
    	ROS_ERROR("ik did not converge: j1: %.3f, j2: %.3f, j3: %.3f, jv: %.3f!", q_cmd(0), q_cmd(1), q_cmd(2), q_cmd(3));
    }
}

} // end namespace


int main( int argc, char **argv)
{
    ros::init(argc, argv, "r2_gaze_controller");
    ros::NodeHandle nodeHandle;
    
    // TODO: Take root and tip names as parameters
    r2_gaze_controller::R2GazeController gazeController(nodeHandle, "waist_center", "vision_center_link");
    if (!gazeController.initialize())
    {
        ROS_ERROR_NAMED("r2_gaze_controller", "could not initialize");
        return -1;
    }
    
    ros::spin();
    return 0;
}
