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


#include "r2_impedance_controller.h"
#include <kdl_parser/kdl_parser.hpp>
#include "pluginlib/class_list_macros.h"
#include "tf_conversions/tf_kdl.h"
#include "tf/transform_datatypes.h"

using namespace std;
using namespace KDL;
using namespace r2_controller_ns;


Eigen::MatrixXd WholeBodyCalc::calcPinv(const Eigen::MatrixXd &a){
using namespace Eigen;
	// see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse#The_general_case_and_the_SVD_method
	const MatrixXd* m;
	MatrixXd t;
	MatrixXd m_pinv;

	// transpose so SVD decomp can work...
	if ( a.rows()<a.cols() ){
		t = a.transpose();
		m = &t;
	} else {
		m = &a;
	}

	// SVD
	//Eigen::JacobiSVD<MatrixXd> svd = m->jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::JacobiSVD<MatrixXd> svd = m->jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
	Eigen::MatrixXd vSingular = svd.singularValues();
	// Build a diagonal matrix with the Inverted Singular values
	// The pseudo inverted singular matrix is easy to compute :
	// is formed by replacing every nonzero entry by its reciprocal (inversing).
	Eigen::MatrixXd vPseudoInvertedSingular(svd.matrixV().cols(),1);
	for (int iRow =0; iRow<vSingular.rows(); iRow++) {
							 // Todo : Put epsilon in parameter
		if ( fabs(vSingular(iRow))<=1e-10 ) {
			vPseudoInvertedSingular(iRow,0)=0.;
		}
		else {
			vPseudoInvertedSingular(iRow,0)=1./vSingular(iRow);
		}
	}
	// A little optimization here
	Eigen::MatrixXd mAdjointU = svd.matrixU().adjoint().block(0,0,vSingular.rows(),svd.matrixU().adjoint().cols());
	// Pseudo-Inversion : V * S * U'
	m_pinv = (svd.matrixV() *  vPseudoInvertedSingular.asDiagonal()) * mAdjointU  ;

	// transpose back if necessary
	if ( a.rows()<a.cols() )
		return m_pinv.transpose();
	
	return m_pinv;
}




bool R2ImpedanceController::init(pr2_mechanism_model::RobotState* robot_state, ros::NodeHandle& n )
{
	boost::lock_guard<boost::mutex> lock(thread_mutex);
	node = n;
	this->robot_state = robot_state;
	kdl_parser::treeFromParam( "robot_description", cc.robot_tree );
	
	int jnt_size = cc.robot_tree.getNrOfJoints();
	robotStateJoints.resize( jnt_size );
	
	
	
	
	cc.init();
	{  //map RobotState joints to KDL::Tree joints
		const SegmentMap& sm = cc.robot_tree.getSegments();
		int x=0;
		for( SegmentMap::const_iterator i = sm.begin(); i!=sm.end(); ++i ){
			const Segment& seg = i->second.segment;
			const Joint& joint = seg.getJoint();
			if( joint.getType() == Joint::None )
				continue;
			cc.name2idx[ joint.getName() ] = x;
			cc.idx2name[x] = joint.getName();
			pr2_mechanism_model::JointState* js = robot_state->getJointState( joint.getName() );
			robotStateJoints[x] = js;
			cc.jntsUpperLimit[x] = js->joint_->limits->upper;
			cc.jntsLowerLimit[x] = js->joint_->limits->lower;
			cc.jntsCenterPoint[x] = (cc.jntsUpperLimit[x] + cc.jntsLowerLimit[x])*.5;
			cc.jntsUpperLimit[x] -= .01;
			cc.jntsLowerLimit[x] += .01;
			
			
			
			cc.desired[x] = cc.jntsCenterPoint[x];
			x++;
			
		}
	}
	load_params(); // need to do this after name2idx is filled
	init_ros_msgs();
	for( int x=0; x<jnt_size; ++x ){
		cc.K[x] = cc.K_high[x];
		cc.D[x] = cc.D_high[x];
	}
	
	{
		const SegmentMap& sm = cc.robot_tree.getSegments();
		int x=0;
		for( SegmentMap::const_iterator i= sm.begin(); i!=sm.end(); ++i ){
			const Segment& seg = i->second.segment;
			const Joint& joint = seg.getJoint();
			if( joint.getType() == Joint::None )
				continue;
			int idx = cc.name2idx[ joint.getName() ];
			cout << "\t" << joint.getName();
			cout << " " << idx;
			cout << " "<< cc.K_high[x] << " " << cc.K_low[x];
			cout << " " << cc.D_high[x] << " " << cc.D_low[x];
			cout << " " << cc.K[x] << " " << cc.D[x] << endl;
			
			++x;
		}
	}
	
	return true;
}
void R2ImpedanceController::CtrlCalc::init(){
	rne_calc.reset(new TreeIdSolver_RNE( robot_tree, KDL::Vector(0,0,-9.8) ));
	

	wbc = WholeBodyCalc( robot_tree );
	jnt_size = robot_tree.getNrOfJoints();
	
	D_high.resize( jnt_size );
	D_low.resize( jnt_size );
	K_high.resize( jnt_size );
	K_low.resize( jnt_size );
	D.resize( jnt_size );
	K.resize( jnt_size );
	desired.resize( jnt_size );
	
	treeJnts.resize( jnt_size );
	treeJntsVel.resize( jnt_size );
	treeJntsAvg.resize( jnt_size );
	treeJntsVelAvg.resize(jnt_size);
	
	jntsUpperLimit.resize( jnt_size );
	jntsLowerLimit.resize( jnt_size );
	jntsCenterPoint.resize( jnt_size );
	torques.resize(jnt_size);
	idx2name.resize(jnt_size);
	cartK_left.resize(6);
	cartD_left.resize(6);
	cartK_right.resize(6);
	cartD_right.resize(6);
	for( int x=0; x<jnt_size; ++x ){
		D_high[x] = 0;
		D_low[x]  = 0;
		K_high[x] = 0;
		K_low[x]  = 0;
		D[x]      = 0;
		K[x]      = 0;
		desired[x]= 0;
		torques(x)= 0;
		treeJnts[x]=0;
		treeJntsVel[x]=0;
	}
	left_cart = false;
	right_cart = false;
	neck_cart = false;
	
	//root_name = "robot_base";
	root_name = "robot_reference";
	

}

///concatenate  second to the end of first
static void append( vector<string>& first, const vector<string>& second ){
	first.insert( first.end(), second.begin(), second.end() );
}
///generate the joint names of the form    base#, where # is [0,count)
static vector<string> createNames( const string& base, int count ){
	vector<string> results(count);
	for( int x=0; x<count; ++x ){
		stringstream ss;
		ss << base << x;
		results[x] = ss.str();
	}
	return results;
} 

static vector<string> createHandNames( const string& a ){
	vector<string> hand;
	{
		vector<string> temp = createNames( a+"_thumb/j",5 );
		hand.push_back( temp[0] );
		// skip X_thumb/j1
		hand.push_back( temp[2] );
		hand.push_back( temp[3] );
		hand.push_back( temp[4] );
	}
	append( hand, createNames( a+"_index/j", 4 ));
	append( hand, createNames( a+"_middle/j", 4 ));
	append( hand, createNames( a+"_ring/j", 3));
	append( hand, createNames( a+"_little/j", 3 ));

	return hand;
}

void R2ImpedanceController::load_params(){
	
	//gains are laid out in the yaml file in these sections
	vector<string> right = createNames( "right/j", 7 );
	vector<string> rightHand = createHandNames( "right" );
	vector<string> left = createNames( "left/j", 7 );
	vector<string> leftHand = createHandNames( "left" );
	vector<string> waist;	waist.push_back( "waist/j0" );
	vector<string> neck = createNames( "neck/j", 3 );
	vector<string> rCart = createNames( "right/x",6);
	vector<string> lCart = createNames( "left/x",6);
	
	///helps put the named values from getGainParams into the correct index
	class Helper{
		std::map<string, int>& map;
		vector<double>& v;
		public:
		Helper( std::map<string, int>& Map, std::vector<double>& V  ):map(Map), v(V){};
		
		void operator()( const vector<string>& names, const vector<double>& values ){
			int size = names.size();
			for( int x=0; x< size; ++x ){
				v[ map[ names[x] ] ] = values[x];
			}
		}
	};
	Helper Khigh( cc.name2idx, cc.K_high );
	Helper Dhigh( cc.name2idx, cc.D_high );
	Helper Klow( cc.name2idx, cc.K_low );
	Helper Dlow( cc.name2idx, cc.D_low );
	
	Khigh( waist, getGainParams( waist, "Waist_Joint_Stiffness_High" ) );
	Dhigh( waist, getGainParams( waist, "Waist_Joint_Damping_High" ));
	Klow( waist, getGainParams( waist, "Waist_Joint_Stiffness_Low" ) );
	Dlow( waist, getGainParams( waist, "Waist_Joint_Damping_Low" ) );
	
	Khigh( neck, getGainParams( neck, "Neck_Joint_Stiffness" ));
	Dhigh( neck, getGainParams( neck, "Neck_Joint_Damping" ));
	Klow( neck, getGainParams( neck, "Neck_Joint_Stiffness" ));
	Dlow( neck, getGainParams( neck, "Neck_Joint_Damping" ));
	
	Khigh( left, getGainParams( left, "LeftArm_Joint_Stiffness_High" ) );
	Dhigh( left, getGainParams( left, "LeftArm_Joint_Damping_High" ) );
	Khigh( leftHand, getGainParams( leftHand, "LeftHand_Joint_Stiffness_High" ));
	Dhigh( leftHand, getGainParams( leftHand, "LeftHand_Joint_Damping_High" ));
	Klow( left, getGainParams( left, "LeftArm_Joint_Stiffness_Low" ));
	Dlow( left, getGainParams( left, "LeftArm_Joint_Damping_Low" ));
	Klow( leftHand, getGainParams( leftHand, "LeftHand_Joint_Stiffness_Low" ));
	Dlow( leftHand, getGainParams( leftHand, "LeftHand_Joint_Damping_Low" ));
	
	Khigh( right, getGainParams( right, "RightArm_Joint_Stiffness_High" ) );
	Dhigh( right, getGainParams( right, "RightArm_Joint_Damping_High" ) );
	Khigh( rightHand, getGainParams( rightHand, "RightHand_Joint_Stiffness_High" ));
	Dhigh( rightHand, getGainParams( rightHand, "RightHand_Joint_Damping_High" ));
	Klow( right, getGainParams( right, "RightArm_Joint_Stiffness_Low" ));
	Dlow( right, getGainParams( right, "RightArm_Joint_Damping_Low" ));
	Klow( rightHand, getGainParams( rightHand, "RightHand_Joint_Stiffness_Low" ));
	Dlow( rightHand, getGainParams( rightHand, "RightHand_Joint_Damping_Low" ));
	
	
	cc.cartK_left = getGainParams( lCart, "Left_Cart_Stiffness" );
	cc.cartD_left = getGainParams( lCart, "Left_Cart_Damping" );
	cc.cartK_right = getGainParams( rCart, "Right_Cart_Stiffness" );
	cc.cartD_right = getGainParams( rCart, "Right_Cart_Damping" );
	
};

vector<double> R2ImpedanceController::getGainParams(const std::vector<string>& param_names, const string& param_name) {
	int param_size = param_names.size();
	vector<double> params; params.reserve( param_size );
	using namespace XmlRpc;
	XmlRpc::XmlRpcValue param_vals;
	if (!node.getParam(param_name, param_vals)) {
		ROS_ERROR("No %s given. (namespace: %s)", param_name.c_str(), node.getNamespace().c_str());
		assert(false);
		return params;
	}
	if (param_size != (int)(param_vals.size())) {
		ROS_ERROR("Incorrect number of %s specified.  (namespace: %s), needs: %d, has %d", param_name.c_str(), node.getNamespace().c_str(), param_size, param_vals.size());
		assert(false);
		return params;
	}
	for (int i = 0; i < param_size; ++i) {
		XmlRpcValue &param_value = param_vals[i];
		if (param_value.getType() != XmlRpcValue::TypeDouble) {
			ROS_ERROR("Array of params names should contain all doubles.  (namespace: %s)",
				node.getNamespace().c_str());
			assert(false);
			return params;
		}
		params.push_back( (double)param_value );
	}
	return params;
}


void R2ImpedanceController::init_ros_msgs(){
	
	left_tip_pose_publisher.reset( new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(node, "left/pose_state", 5));
	right_tip_pose_publisher.reset(new realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>(node, "right/pose_state", 5));

	left_pose_error_publisher.reset( new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(node, "left/twist_error", 5));
	right_pose_error_publisher.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Twist>(node, "right/twist_error", 5));

	// subscribe to pose commands
	left_joint_command_sub  = node.subscribe("left_arm/joint_command",  5, &R2ImpedanceController::joint_left_command,  this);
	right_joint_command_sub = node.subscribe("right_arm/joint_command", 5, &R2ImpedanceController::joint_right_command, this);
	neck_joint_command_sub  = node.subscribe("neck/joint_command",      5, &R2ImpedanceController::joint_neck_command,  this);
	waist_joint_command_sub = node.subscribe("waist/joint_command",     5, &R2ImpedanceController::joint_waist_command, this);

	//left_pose_command_sub = node.subscribe("left/pose_command", 5, &R2ImpedanceController::pose_left_command, this);
	left_pose_command_sub.subscribe(node, "left/pose_command", 1);
	left_pose_command_filter.reset(new tf::MessageFilter<geometry_msgs::PoseStamped>(left_pose_command_sub, tfListener, cc.root_name, 10, node ));
	left_pose_command_filter->registerCallback(boost::bind(&R2ImpedanceController::pose_left_command, this, _1));

	right_pose_command_sub.subscribe(node, "right/pose_command", 1);
	right_pose_command_filter.reset(new tf::MessageFilter<geometry_msgs::PoseStamped>(right_pose_command_sub, tfListener, cc.root_name, 10, node));
	right_pose_command_filter->registerCallback(boost::bind(&R2ImpedanceController::pose_right_command, this, _1));

	
	set_gains_sub = node.subscribe("set_gains", 3, &R2ImpedanceController::set_gains, this );
	gains_publisher.reset( new realtime_tools::RealtimePublisher<r2_msgs::Gains>(node, "gains", 5 ));
	
	
	srv_set_joint_mode = node.advertiseService("set_joint_mode", &R2ImpedanceController::set_joint_mode, this);
	srv_set_tip_name   = node.advertiseService("set_tip_name",   &R2ImpedanceController::set_tip_name,   this);

	srv_set_power = node.advertiseService("power", &R2ImpedanceController::set_power, this);
	srv_set_servo = node.advertiseService("servo", &R2ImpedanceController::set_servo, this);
	
	
	
}
//since JointState actually names the joints, we don't need seperate functions for left/right/neck/waist joints
// if desired, could check for illegal names ( left shouldn't change right values, etc ) but really should just
// consolidate to one topic 
void R2ImpedanceController::joint_left_command(const sensor_msgs::JointState::ConstPtr& msg ){ joint_command(msg);}
void R2ImpedanceController::joint_right_command(const sensor_msgs::JointState::ConstPtr& msg ){joint_command(msg);}
void R2ImpedanceController::joint_neck_command(const sensor_msgs::JointState::ConstPtr& msg ){joint_command(msg);}
void R2ImpedanceController::joint_waist_command(const sensor_msgs::JointState::ConstPtr& msg ){joint_command(msg);}
void R2ImpedanceController::pose_left_command(const geometry_msgs::PoseStamped::ConstPtr& msg){
	
	Frame f = transformPoseMsg(msg);
	boost::lock_guard<boost::mutex> lock(thread_mutex);
	cc.leftCmd[0] = f.p[0];
	cc.leftCmd[1] = f.p[1];
	cc.leftCmd[2] = f.p[2];
	double& w = cc.leftCmd[3];
	double& x = cc.leftCmd[4];
	double& y = cc.leftCmd[5];
	double& z = cc.leftCmd[6];
	f.M.GetQuaternion(x,y,z,w);
}
void R2ImpedanceController::pose_right_command(const geometry_msgs::PoseStamped::ConstPtr& msg){
	Frame f = transformPoseMsg(msg);
	boost::lock_guard<boost::mutex> lock(thread_mutex);
	cc.rightCmd[0] = f.p[0];
	cc.rightCmd[1] = f.p[1];
	cc.rightCmd[2] = f.p[2];
	double& w = cc.rightCmd[3];
	double& x = cc.rightCmd[4];
	double& y = cc.rightCmd[5];
	double& z = cc.rightCmd[6];
	f.M.GetQuaternion(x,y,z,w);
}
KDL::Frame R2ImpedanceController::transformPoseMsg(const geometry_msgs::PoseStamped::ConstPtr& msg){
	Frame frame;
	tf::Stamped<tf::Pose> pose_stamped;
	tf::poseStampedMsgToTF(*msg, pose_stamped);
	tfListener.transformPose( cc.root_name, pose_stamped, pose_stamped );
	tf::PoseTFToKDL(pose_stamped, frame );
	
	return frame;
	
}
void R2ImpedanceController::set_gains(const r2_msgs::Gains::ConstPtr& msg ){
	boost::lock_guard<boost::mutex> lock(thread_mutex);
	size_t size = msg->joint_names.size();
	if( size != msg->K.size() || size!= msg->D.size() ){
		cout << "WARNING: discarding Gains msg due to mismatch in size.  joint_names: " << size << " values: " << msg->K.size() <<" " <<msg->D.size() << endl;
		return;
	}
	///need to handle cart gains differently since they aren't indexed into the usual arrays
	static const string cart_left_names[] = { "cart_left_0", 
											  "cart_left_1",
											  "cart_left_2",
											  "cart_left_3",
											  "cart_left_4",
											  "cart_left_5" };
	static const string cart_right_names[] = {  "cart_right_0",
												"cart_right_1",
												"cart_right_2",
												"cart_right_3",
												"cart_right_4",
												"cart_right_5"};
	 
	
	
	for( size_t x=0; x<size; ++x ){
		map< string, int>::const_iterator i = cc.name2idx.find( msg->joint_names[x] );
		if( i != cc.name2idx.end() ){
			cc.K[i->second] = msg->K[x];
			cc.D[i->second] = msg->D[x];
		} else {
			for( int y=0; y<6; ++y ){
				if( msg->joint_names[x] == cart_left_names[y] ){
					cc.cartK_left[y] = msg->K[x];
					cc.cartD_left[y] = msg->D[x];
					break;
				}
				if( msg->joint_names[x] == cart_right_names[y] ){
					cc.cartK_right[y] = msg->K[x];
					cc.cartD_right[y] = msg->D[x];
					break;
				}
			}
		}
	}
	
}


void R2ImpedanceController::joint_command(const sensor_msgs::JointState::ConstPtr& msg){
	boost::lock_guard<boost::mutex> lock(thread_mutex);
	//need to handle non-independent joint differently
	const static string not_independent[] = { "left_index/j3",
											  "left_middle/j3",
											  "left_ring/j1",
											  "left_ring/j2",
											  "left_little/j1",
											  "left_little/j2",
											  "right_index/j3",
											  "right_middle/j3",
											  "right_ring/j1",
											  "right_ring/j2",
											  "right_little/j1",
											  "right_little/j2" };
	const static int ni_count = 12;										  
											  
	
	
	for( size_t i=0; i< msg->position.size(); ++i ){
		const string& name = msg->name[i];
		double p = msg->position[i];
		bool found = false;
		for( int x=0; x< ni_count; ++x ){
			if( name == not_independent[x] ){
				found = true;
				break;
			}
		}
		if( found )
			continue;
		
		map<string,int>::const_iterator i = cc.name2idx.find(name);
		if( i != cc.name2idx.end() ){
			if( p > cc.jntsUpperLimit[ i->second ] )
				p = cc.jntsUpperLimit[ i->second ];
			if( p < cc.jntsLowerLimit[ i->second ] )
				p = cc.jntsLowerLimit[ i->second ];
				
			cc.desired[ i->second ] = p;
		}
		if( name == "left_index/j2" ){
			cc.desired[ cc.name2idx[ "left_index/j3" ] ] = p;
		} else if( name == "left_middle/j2" ){
			cc.desired[ cc.name2idx[ "left_middle/j3" ] ] = p;
		} else if( name == "left_ring/j0" ){
			cc.desired[ cc.name2idx[ "left_ring/j0" ] ] = p*.5;
			cc.desired[ cc.name2idx[ "left_ring/j1" ] ] = p*.5;
			cc.desired[ cc.name2idx[ "left_ring/j2" ] ] = p*.5;
		} else if( name == "left_little/j0" ){
			cc.desired[ cc.name2idx[ "left_little/j0" ] ] = p*.5;
			cc.desired[ cc.name2idx[ "left_little/j1" ] ] = p*.5;
			cc.desired[ cc.name2idx[ "left_little/j2" ] ] = p*.5;
		} else if( name == "right_index/j2" ){
			cc.desired[ cc.name2idx[ "right_index/j3" ] ] = p;
		} else if( name == "right_middle/j2" ){
			cc.desired[ cc.name2idx[ "right_middle/j3" ] ] = p;
		} else if( name == "right_ring/j0" ){
			cc.desired[ cc.name2idx[ "right_ring/j0" ] ] = p*.5;
			cc.desired[ cc.name2idx[ "right_ring/j1" ] ] = p*.5;
			cc.desired[ cc.name2idx[ "right_ring/j2" ] ] = p*.5;
		} else if( name == "right_little/j0" ){
			cc.desired[ cc.name2idx[ "right_little/j0" ] ] = p*.5;
			cc.desired[ cc.name2idx[ "right_little/j1" ] ] = p*.5;
			cc.desired[ cc.name2idx[ "right_little/j2" ] ] = p*.5;
		}
	}
}

bool R2ImpedanceController::set_joint_mode(r2_msgs::SetJointMode::Request  &req,  r2_msgs::SetJointMode::Response &res ){ 
	boost::lock_guard<boost::mutex> lock(thread_mutex);
	class Local{
		public:
		static void setDesired( vector<double>& desired, const vector<double>& current, const TreeChain& tc ){
			int size = tc.size();
			for( int x=0; x< size; ++x ){
				int idx = tc.treeIdx(x);
				desired[idx] = current[idx];
			}
		}
	};
	if( req.arm_name == "left" && cc.left_cart ){
		cc.left_cart = false;
		Local::setDesired( cc.desired, cc.treeJnts, cc.left );
	}
	if( req.arm_name == "right" && cc.right_cart ){
		cc.right_cart = false;
		Local::setDesired( cc.desired, cc.treeJnts, cc.right );
	}
	if( req.arm_name == "neck" && cc.neck_cart ){
		cc.neck_cart = false;
		Local::setDesired( cc.desired, cc.treeJnts, cc.neck );
	}
		
	res.result = true;
	cc.reactivate();
	
	return res.result;
}
bool R2ImpedanceController::set_tip_name(r2_msgs::SetTipName::Request &req,  r2_msgs::SetTipName::Response &res ){
	boost::lock_guard<boost::mutex> lock(thread_mutex);
	if( req.arm_name == "left" ){
		cc.left.init( cc.robot_tree, cc.root_name, req.tip_name, cc.cartK_left, cc.cartD_left );
		cc.activate( cc.left, cc.left_cart, cc.leftCmd );
        res.result = true;
	} else if( req.arm_name == "right" ){
		cc.right.init( cc.robot_tree, cc.root_name, req.tip_name, cc.cartK_right, cc.cartD_right );
		cc.activate( cc.right, cc.right_cart, cc.rightCmd );
        res.result = true;
	} else {
		res.result = false;
	}
	return res.result;
}
bool R2ImpedanceController::set_power(r2_msgs::Power::Request &req,  r2_msgs::Power::Response &res ){ res.status = true; return true;}
bool R2ImpedanceController::set_servo(r2_msgs::Servo::Request &req,  r2_msgs::Servo::Response &res ){ res.status = true; return true;}

void R2ImpedanceController::CtrlCalc::activate( TreeChain& tc, bool& flag, Eigen::Matrix<double,7,1>& pose_cmd ){
	flag = true;
	
	for( int x=0; x<tc.size(); ++x ){
		int idx = tc.treeIdx(x);
		K[ idx ] = K_low[ idx ];
		D[ idx ] = D_low[ idx ];
	}
	pose_cmd = tc.fk( treeJnts );
}
void R2ImpedanceController::CtrlCalc::reactivate(){
	for( int x=0; x<jnt_size; ++x ){
		K[x] = K_high[x];
		D[x] = D_high[x];
	}
	bool dummy_flag;
	Eigen::Matrix<double,7,1> dummy_cmd;
	if( left_cart )
		activate( left, dummy_flag, dummy_cmd );
	if( right_cart )
		activate( right, dummy_flag, dummy_cmd );
	if( neck_cart )
		activate( neck, dummy_flag, dummy_cmd );
}


KDL::JntArray R2ImpedanceController::CtrlCalc::jointKCmd(const vector<double>& q){
	
	KDL::JntArray result( jnt_size );
	for( int x=0; x< jnt_size; ++x ){
		result(x) = (K[x] * (desired[x] - q[x]));
		//result(x) = ( 0-q[x] ) - dq[x];
		//cout << x << " \t " << q[x] << " \t " << dq[x] << endl;
	}
	return result;
}
KDL::JntArray R2ImpedanceController::CtrlCalc::jointDCmd(const vector<double>& dq){
	
	KDL::JntArray result( jnt_size );
	for( int x=0; x< jnt_size; ++x ){
		result(x) = -(D[x] * dq[x]);
		//result(x) = ( 0-q[x] ) - dq[x];
		//cout << x << " \t " << q[x] << " \t " << dq[x] << endl;
	}
	return result;
}


void R2ImpedanceController::update(){
	boost::lock_guard<boost::mutex> lock(thread_mutex);
	
	///update section
	ros::Time last_time = time;
	time = robot_state->getTime();
	double deltaT = (time-last_time).toSec();
	if( deltaT < .000001 )
		deltaT = .000001;
	//const double qd_scale_factor = 1;
	
	
	cc.wbc.reset();
	//cout << deltaT << " | ";
	{
		///read current joint values
		for( int x=0; x<cc.jnt_size; ++x ){
			double p = robotStateJoints[x]->position_;
			double v = robotStateJoints[x]->velocity_;
			//double v2 = (p-treeJnts[x])/deltaT;
			//if( fabs(v-v2) < .01 )
			//	cout << ".";
			//else
			//	cout << v << " " << v2 <<" [" << p << " " << idx2name[x] <<"] \t";
			///seems like j5,j6 have some instability in position. explicit calculation seems to cause large swings in velocity
			if( v == v ){	/// we can get NaN here for some reason
				if( v < -1.0 )
					v = -1.0;
				else if( v > 1.0 )
					v = 1.0;
				
				
				cc.treeJntsVelAvg[x] = v;
			} else
				cc.treeJntsVelAvg[x] = 0;{
			//	v = (p - treeJnts[x])/deltaT;
			//	if( v == v )
			//		treeJntsVel[x] = v;
			}
			cc.treeJntsAvg[x] = p;
			cc.treeJnts[x] = cc.treeJntsAvg[x];
			cc.treeJntsVel[x] = cc.treeJntsVelAvg[x];
		}
	}
	
	
	/// calculate section
	cc.calculate();
		///send commands to joints
	{
		for( int x=0; x<cc.jnt_size; ++x ){
			robotStateJoints[x]->commanded_effort_ = cc.torques(x);
		}
		robot_state->enforceSafety();
	}
	static int loopCnt = 0;
	if( loopCnt++ > 50 ){
		loopCnt=0;
		publish_msgs();
	}
}
void R2ImpedanceController::CtrlCalc::calculate(){	
	
	KDL::SetToZero(torques);
	Eigen::VectorXd left_t = Eigen::VectorXd::Zero(jnt_size);
	Eigen::VectorXd right_t = Eigen::VectorXd::Zero(jnt_size);
	
	
	Wrenches wrenches( robot_tree.getNrOfSegments() );
	vector<double> zeros( treeJnts.size() );
	for( int x=0; x<treeJnts.size(); ++x )
		zeros[x] = 0;
	
	rne_calc->CartToJnt( treeJnts, treeJntsVel, zeros, wrenches, torques);
	//for( int x=0; x<treeJnts.size(); ++x ){
	//	cout << torques(x) << " ";
	//}
	//cout << endl;
//	int CartToJnt(const JntArray &q, const JntArray &q_dot, const JntArray &q_dotdot, const Wrenches& f_ext,JntArray &torques);

	
	if( left_cart ){
		torques.data += wbc.project( left.moveCart(leftCmd, treeJnts, treeJntsVel ), left ).data;
	}
	if( right_cart ){
		torques.data += wbc.project( right.moveCart(rightCmd, treeJnts, treeJntsVel), right ).data;
	}
	if( neck_cart )
		torques.data += wbc.project( neck.moveCart(neckCmd, treeJnts, treeJntsVel), neck ).data;
	
	torques.data += wbc.project( jointKCmd( treeJnts ) ).data;
	torques.data += wbc.project(jointDCmd( treeJntsVel ) ).data;
	//torques.data += jointDCmd( treeJntsVel).data; 



	
}

void R2ImpedanceController::publish_msgs(){
	if( gains_publisher && gains_publisher->trylock() ){
		r2_msgs::Gains& msg = gains_publisher->msg_;
		msg.joint_names.clear();
		msg.K.clear();
		msg.D.clear();
		for( map<string,int>::const_iterator i=cc.name2idx.begin(); i!=cc.name2idx.end(); ++i ){
			msg.joint_names.push_back( i->first );
			msg.K.push_back( cc.K[ i->second ] );
			msg.D.push_back( cc.D[ i->second ] );
		}
		string cleft = "cart_left_";
		string cright = "cart_right_";
		for( int x=0; x<6; ++x ){
			stringstream ss;
			ss << x;
			msg.joint_names.push_back( cleft + ss.str() );
			msg.K.push_back( cc.cartK_left[x] );
			msg.D.push_back( cc.cartD_left[x] );
			
			msg.joint_names.push_back( cright + ss.str() );
			msg.K.push_back( cc.cartK_right[x] );
			msg.D.push_back( cc.cartD_right[x] );
		}
		gains_publisher->unlockAndPublish();
	}
	if( cc.left_cart ){
		Eigen::Matrix<double, 7, 1> fk = cc.left.fk(cc.treeJnts);
		KDL::Frame f1( KDL::Rotation::Quaternion( fk[4], fk[5], fk[6], fk[3] ), KDL::Vector( fk[0], fk[1], fk[2] ) );
		KDL::Frame f2( KDL::Rotation::Quaternion( cc.leftCmd[4], cc.leftCmd[5], cc.leftCmd[6], cc.leftCmd[3] ), KDL::Vector( cc.leftCmd[0], cc.leftCmd[1], cc.leftCmd[2] ) );
		
		KDL::Twist twist = KDL::diff( f1, f2 );	
	
		if( left_pose_error_publisher && left_pose_error_publisher->trylock() ){
			left_pose_error_publisher->msg_.linear.x = twist.vel(0);
			left_pose_error_publisher->msg_.linear.y = twist.vel(1);
			left_pose_error_publisher->msg_.linear.z = twist.vel(2);
			left_pose_error_publisher->msg_.angular.x = twist.rot(0);
			left_pose_error_publisher->msg_.angular.y = twist.rot(1);
			left_pose_error_publisher->msg_.angular.z = twist.rot(2);
			left_pose_error_publisher->unlockAndPublish();
		}
		if( left_tip_pose_publisher && left_tip_pose_publisher->trylock() ){
			tf::Pose tmp;
			tf::PoseKDLToTF( f1, tmp );
			poseStampedTFToMsg( tf::Stamped<tf::Pose>( tmp, ros::Time::now(), cc.root_name), left_tip_pose_publisher->msg_ );
			left_tip_pose_publisher->unlockAndPublish();
		}
	}
	if( cc.right_cart ){
		Eigen::Matrix<double, 7, 1> fk = cc.right.fk(cc.treeJnts);
		KDL::Frame f1( KDL::Rotation::Quaternion( fk[4], fk[5], fk[6], fk[3] ), KDL::Vector( fk[0], fk[1], fk[2] ) );
		KDL::Frame f2( KDL::Rotation::Quaternion( cc.rightCmd[4], cc.rightCmd[5], cc.rightCmd[6], cc.rightCmd[3] ), KDL::Vector( cc.rightCmd[0], cc.rightCmd[1], cc.rightCmd[2] ) );
		
		KDL::Twist twist = KDL::diff( f1, f2 );	
		
		if( right_pose_error_publisher && right_pose_error_publisher->trylock() ){
			right_pose_error_publisher->msg_.linear.x  = twist.vel(0);
			right_pose_error_publisher->msg_.linear.y  = twist.vel(1);
			right_pose_error_publisher->msg_.linear.z  = twist.vel(2);
			right_pose_error_publisher->msg_.angular.x = twist.rot(0);
			right_pose_error_publisher->msg_.angular.y = twist.rot(1);
			right_pose_error_publisher->msg_.angular.z = twist.rot(2);
			right_pose_error_publisher->unlockAndPublish();
		}

		if( right_tip_pose_publisher && right_tip_pose_publisher->trylock() ){
			tf::Pose tmp;
			tf::PoseKDLToTF( f1, tmp );
			poseStampedTFToMsg( tf::Stamped<tf::Pose>( tmp, ros::Time::now(), cc.root_name), right_tip_pose_publisher->msg_ );
			right_tip_pose_publisher->unlockAndPublish();
		}
	}
	
}




PLUGINLIB_DECLARE_CLASS(r2_controllers_gazebo, R2ImpedanceController, r2_controller_ns::R2ImpedanceController, pr2_controller_interface::Controller)



