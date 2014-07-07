/*
 * Copyright (c) 2012, General Motors.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Darren Earl, Stephen Hart
 */

#pragma once

#include "ros/ros.h"
#include <realtime_tools/realtime_publisher.h>
#include <tf/message_filter.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/jacobian.hpp"
#include "kdl/chainjnttojacsolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainidsolver_recursive_newton_euler.hpp"
#include "treeidsolver_recursive_newton_euler.hpp"
#include <vector>

#include <Eigen/Geometry>
#include <boost/thread/mutex.hpp>

#include <message_filters/subscriber.h>
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/chain.h>
#include <realtime_tools/realtime_publisher.h>
#include <algorithm>

///messages
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nasa_r2_common_msgs/Gains.h>
#include <nasa_r2_common_msgs/SetTipName.h>
#include <nasa_r2_common_msgs/SetJointMode.h>
#include <nasa_r2_common_msgs/Power.h>
#include <nasa_r2_common_msgs/Servo.h>
#include <nasa_r2_common_msgs/PoseTwistStamped.h>

#include "TreeChain.h"
#include "WholeBodyCalc.h"



namespace r2_controller_ns {
class R2ImpedanceController: public pr2_controller_interface::Controller{
	
	//ros messaging
	ros::NodeHandle node;
	tf::TransformListener tfListener;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> >	left_tip_pose_publisher;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist> >		left_pose_error_publisher;
	ros::Subscriber 									joint_command_sub;
	ros::Subscriber 									left_joint_command_sub;
	
	message_filters::Subscriber<geometry_msgs::PoseStamped> 				left_pose_command_sub;
	boost::scoped_ptr<tf::MessageFilter<geometry_msgs::PoseStamped> >			left_pose_command_filter;
	message_filters::Subscriber<nasa_r2_common_msgs::PoseTwistStamped> 					left_pose_vel_command_sub;
	boost::scoped_ptr<tf::MessageFilter<nasa_r2_common_msgs::PoseTwistStamped> >			left_pose_vel_command_filter;
	
	boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> >	right_tip_pose_publisher;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Twist> >		right_pose_error_publisher;
	ros::Subscriber 									right_joint_command_sub;
	
	message_filters::Subscriber<geometry_msgs::PoseStamped> 				right_pose_command_sub;
	boost::scoped_ptr<tf::MessageFilter<geometry_msgs::PoseStamped> >			right_pose_command_filter;
	message_filters::Subscriber<nasa_r2_common_msgs::PoseTwistStamped>	 				right_pose_vel_command_sub;
	boost::scoped_ptr<tf::MessageFilter<nasa_r2_common_msgs::PoseTwistStamped> >			right_pose_vel_command_filter;
	
	
	
	ros::Subscriber neck_joint_command_sub;
	ros::Subscriber waist_joint_command_sub;
	
	ros::Subscriber set_gains_sub;
	boost::scoped_ptr<realtime_tools::RealtimePublisher<nasa_r2_common_msgs::Gains> > gains_publisher;
	
	ros::ServiceServer srv_set_joint_mode;
	ros::ServiceServer srv_set_tip_name;
	ros::ServiceServer srv_set_power;
	ros::ServiceServer srv_set_servo;
	public:
	class CtrlCalc{	///non-ros subclass which performs the torque calculations
	public:
		///base of chains, currently both the same
		std::string root_name;
	
		//guess neck isn't used yet
		KDL::Tree robot_tree;
		TreeChain left;
		TreeChain right;
		TreeChain neck;
	
		///desired pose [cartPos(x,y,z), quaternion(w,x,y,z) ]
		Eigen::Matrix<double,7,1> leftCmd;
		Eigen::Matrix<double,7,1> rightCmd;
		Eigen::Matrix<double,7,1> neckCmd;
		KDL::Twist leftVelCmd;
		KDL::Twist rightVelCmd;
		KDL::Twist neckVelCmd;
		
		bool left_cart; 	//< flag for left cartesian mode
		bool left_cart_vel;	//< flag for using velocity in left cart mode
		bool right_cart;	//< flag for right cartesian mode
		bool right_cart_vel;//< flag for using velocity in right cart mode
		bool neck_cart;		//< flag for neck cartesian mode
		bool neck_cart_vel;	//< flag for neck velocity in neck cart mode
		std::vector<int> joint_pos_control; //< flags for p control on joint pos
		std::vector<int> joint_vel_control; //< flags for adjusting d control to joint vel
		
		WholeBodyCalc wbc; //< performs nullspace calculations
	
		//gains
		std::vector<double> D_high;		//< derivative gains 
		std::vector<double> D_low;			//< derivative gains in cart mode
		std::vector<double> K_high;		//< proportional gains
		std::vector<double> K_low;			//< proportional gains in cart mode
		std::vector<double> D;				//< current proportional gain value
		std::vector<double> K;				//< current proportional gain value
		std::vector<double> desired; 		//< desired joint positions
		std::vector<double> desiredVel;	//< desired joint velocity
		int jnt_size;						//< number of joints in tree
	
		std::vector<double> cartK_left;	//< cartesian gains left
		std::vector<double> cartK_right;	//< cartesian gains right
		std::vector<double> cartD_left;	//< cartesian gains left
		std::vector<double> cartD_right;	//< cartesian gains right
	
	
		///median filter for noisy values
		template<int _N>
		class AvgV{
			double data[_N];
			mutable double sorted[_N];
			mutable bool update;
			static const int N = _N;
			//static const bool EVEN = ((N+1)/2 == (N/2));
			int idx;
		public:
			AvgV():update(false),idx(0){}
			operator double()const{
				if( update ){
					update = false;
					for(int x=0; x<N; ++x)
						sorted[x] = data[x];
					std::sort(sorted,sorted+N);
				}
				return sorted[N/2]; // if N==3, N/2 == 1; if N==4, N/2 == 2; both are fine
			}
			void operator=(double in){
				data[idx++] = in;
				if( idx >= N )
					idx =0;
				update = true;
			}
		};
		std::vector< AvgV<3> >treeJntsAvg;
		std::vector< AvgV<3> >treeJntsVelAvg;
	
		std::vector<double> treeJnts;	//<conveniance variable, joint positions
		std::vector<double> treeJntsVel;//<conveniance variable, joint velocity
		std::vector<double> jntsUpperLimit;//< upper limit
		std::vector<double> jntsLowerLimit;//< lower limit
		std::vector<double> jntsCenterPoint;//< middle point
		std::map< std::string, int> name2idx;	//< name to index lookup map
		std::vector<std::string> idx2name;
	
		KDL::JntArray torques;
		
		boost::scoped_ptr<KDL::TreeIdSolver> rne_calc;
		
		KDL::JntArray jointKCmd(const std::vector<double>& q);
		KDL::JntArray jointDCmd(const std::vector<double>& qd);
		
		///sets cartesian mode for TreeChain, pose_cmd is filled with current position
		void activate( TreeChain& tc, bool& flag, Eigen::Matrix<double,7,1>& pose_cmd );
		/// sets gains based on which cartesian modes are active
		void reactivate();
		
		void init(double gravity[3]);
		void calculate();
	};
	private:
	CtrlCalc cc;
	
	
	
	//joints have a unique index value which is preserved across these arrays
	std::vector<pr2_mechanism_model::JointState*> robotStateJoints; //< queries robot state and commands joints

	
	pr2_mechanism_model::RobotState* robot_state; //< 
	ros::Time time;
	
	// ros message functions
	void init_ros_msgs();
	void publish_msgs();
	void joint_left_command(const sensor_msgs::JointState::ConstPtr& msg );
	void joint_right_command(const sensor_msgs::JointState::ConstPtr& msg );
	void joint_neck_command(const sensor_msgs::JointState::ConstPtr& msg );
	void joint_waist_command(const sensor_msgs::JointState::ConstPtr& msg );
	void joint_command( const sensor_msgs::JointState::ConstPtr& msg );
	
	void joint_command_entry( const std::string& name, double value, std::vector<double>& desired );
	void joint_command_entry( const std::string& name, bool value, std::vector<int>& desired );
	
	void pose_left_command(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void pose_right_command(const geometry_msgs::PoseStamped::ConstPtr& msg);

	void pose_vel_left_command(const nasa_r2_common_msgs::PoseTwistStamped::ConstPtr& msg );
	void pose_vel_right_command(const nasa_r2_common_msgs::PoseTwistStamped::ConstPtr& msg );
	void pose_vel_command_inner(	const nasa_r2_common_msgs::PoseTwistStamped::ConstPtr& msg,
					Eigen::Matrix<double,7,1>& cmd,
					KDL::Twist& velCmd,
					bool& cart_vel );

	KDL::Frame transformPoseMsg(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void set_gains(const nasa_r2_common_msgs::Gains::ConstPtr& msg );

	// service functions
	bool set_joint_mode(nasa_r2_common_msgs::SetJointMode::Request  &req,  nasa_r2_common_msgs::SetJointMode::Response &res );
	bool set_tip_name(nasa_r2_common_msgs::SetTipName::Request &req,  nasa_r2_common_msgs::SetTipName::Response &res );
	bool set_power(nasa_r2_common_msgs::Power::Request &req,  nasa_r2_common_msgs::Power::Response &res );
	bool set_servo(nasa_r2_common_msgs::Servo::Request &req,  nasa_r2_common_msgs::Servo::Response &res );
	
	
	
	///loads gains
	void load_params();
	
	///load gains from yaml file, used in load_params
	std::vector<double> getGainParams(const std::vector<std::string>& param_names, const std::string& param_name);
	

	
	boost::mutex thread_mutex;
	
public:
	
	
	
	bool init(pr2_mechanism_model::RobotState* robot_state, ros::NodeHandle& n );
	
	/// occurs before first update() call
	virtual void starting(){
		time = robot_state->getTime();
	}
	/// performs one iteration of control, main function 
	virtual void update();
	};
}
