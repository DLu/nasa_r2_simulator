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

///calculates torque commands  (using jacobian transpose ) for a chain and then returns results
/// applicable for the full tree
class TreeChain{
	std::vector<int> chain2Tree; // hash table to transform chain index to tree index
	KDL::Chain chain;
	KDL::JntArray jnts;
	KDL::JntArrayVel jntsVel;
	KDL::JntArray result;
	KDL::JntArray full_result;
	KDL::Jacobian J;	// jacobian sized for just the chain
	KDL::Jacobian fullJ;	// jacobian sized for the entire tree, zeros in rows/columns not in the chain
	//typedef Eigen::Matrix<double, 6, 1> Vector6d;
	std::vector<double>* pK; //ptr to get immediate updates to gains
	std::vector<double>* pD; //ptr to get immediate updates to gains
	
	int jnt_size;
	int tree_size;
	
	void update( const std::vector<double>& treeJnts, const std::vector<double>& treeJntsVel ){
		for( int x=0; x<jnt_size; ++x ){
			int idx = chain2Tree[x];
			jnts(x) = treeJnts[ idx ];
			jntsVel.q(x) = jnts(x);
			jntsVel.qdot(x) = treeJntsVel[ idx ];
		}
		KDL::ChainJntToJacSolver solver(chain);
		solver.JntToJac( jnts, J );
		for( int y=0; y<jnt_size; ++y ){
			int iy = chain2Tree[y];
			for( int x=0; x<6; ++x ){
				fullJ( x, iy ) = J( x, y );
			}
		}
	}
	
	public:
	int size()const{ return jnt_size; }
	int treeIdx( int chainIdx )const{
		return chain2Tree[ chainIdx ];
	}
	
	const KDL::Jacobian& getJ()const{ return fullJ; }
	TreeChain(): jnt_size(0), tree_size(0){}
	void init( const KDL::Tree& tree, const std::string& root, const std::string& tip, std::vector<double>& Kgains, std::vector<double>& Dgains){
		bool r = tree.getChain( root, tip, chain );
		(void)r;
		assert(r);
		jnt_size = chain.getNrOfJoints();
		tree_size = tree.getNrOfJoints();
		
		jnts.resize(jnt_size);
		jntsVel.resize(jnt_size);
		
		result.resize(jnt_size);
		KDL::SetToZero(result);
		
		full_result.resize(tree_size);
		KDL::SetToZero(full_result);
		
		J.resize(jnt_size);
		fullJ.resize(tree_size);
		SetToZero(fullJ);
		
		pK = &Kgains;
		pD = &Dgains;
		//for( int x=0; x<6; ++x ){
		//	K[x] = Kgains[x];
		//	D[x] = Dgains[x];
		//}

		chain2Tree.resize(jnt_size);
		const KDL::SegmentMap& sm = tree.getSegments();
		
		int tj=0;
		for( KDL::SegmentMap::const_iterator i= sm.begin(); i!=sm.end(); ++i ){
			const KDL::Segment& seg = i->second.segment;
			const KDL::Joint& joint = seg.getJoint();
			if( joint.getType() == KDL::Joint::None )
				continue;
			int cj =0;
			const int segCnt = (int)chain.getNrOfSegments();
			for( int x=0; x<segCnt; ++x ){
				const KDL::Segment& c_seg = chain.getSegment(x);
				if( c_seg.getJoint().getType() == KDL::Joint::None )
					continue;
				if( c_seg.getJoint().getName() == joint.getName() ){
					chain2Tree[cj] = tj;
					break;
				}
				++cj;
			}
			++tj;
		}
	}
	
	const KDL::JntArray& moveCart( Eigen::Matrix<double, 7, 1>& cartCmd, const std::vector<double>& treeJnts, const std::vector<double>& treeJntsVel ){
		update( treeJnts, treeJntsVel );
		
		Eigen::Matrix<double, 7, 1> current = fk();
		Eigen::Matrix<double, 6, 1> v = fk_vel();
		
		KDL::Frame f1( KDL::Rotation::Quaternion( current[4], current[5], current[6], current[3] ), KDL::Vector( current[0], current[1], current[2] ) );
		KDL::Frame f2( KDL::Rotation::Quaternion( cartCmd[4], cartCmd[5], cartCmd[6], cartCmd[3] ), KDL::Vector( cartCmd[0], cartCmd[1], cartCmd[2] ) );
		
		KDL::Twist twist = KDL::diff( f1, f2 );		
		Eigen::Matrix<double, 6, 1> delta;
		
		std::vector<double>& K = *pK;
		std::vector<double>& D = *pD;
		
		delta << twist.vel(0) * K[0] - v[0]*D[0], 
				 twist.vel(1) * K[1] - v[1]*D[1], 
				 twist.vel(2) * K[2] - v[2]*D[2], 
				 twist.rot(0) * K[3] - v[3]*D[3], 
				 twist.rot(1) * K[4] - v[4]*D[4], 
				 twist.rot(2) * K[5] - v[5]*D[5];
		
		result.data = J.data.transpose()*delta;
		
		for( int x=0; x<jnt_size; ++x ){
			full_result( chain2Tree[x] ) = result( x );
		}
		return full_result;
	}
	const KDL::JntArray& moveCart( KDL::Twist& cartVelCmd, const std::vector<double>& treeJnts, const std::vector<double>& treeJntsVel ){
		update( treeJnts, treeJntsVel );
		
		Eigen::Matrix<double, 6, 1> v = fk_vel();
		Eigen::Matrix<double, 6, 1> delta;
		std::vector<double>& D = *pD;
		delta << (cartVelCmd.vel(0) - v[0] )*D[0], 
				 (cartVelCmd.vel(1) - v[1] )*D[1], 
				 (cartVelCmd.vel(2) - v[2] )*D[2], 
				 (cartVelCmd.rot(0) - v[3] )*D[3], 
				 (cartVelCmd.rot(1) - v[4] )*D[4], 
				 (cartVelCmd.rot(2) - v[5] )*D[5];
		result.data = J.data.transpose()*delta;
		for( int x=0; x<jnt_size; ++x ){
			full_result( chain2Tree[x] ) = result( x );
		}
		return full_result;
	}
	const KDL::JntArray& moveCart( Eigen::Matrix<double, 7, 1>& cartCmd, KDL::Twist& cartVelCmd, const std::vector<double>& treeJnts, const std::vector<double>& treeJntsVel ){
		update( treeJnts, treeJntsVel );
		
		Eigen::Matrix<double, 7, 1> current = fk();
		Eigen::Matrix<double, 6, 1> v = fk_vel();
		
		KDL::Frame f1( KDL::Rotation::Quaternion( current[4], current[5], current[6], current[3] ), KDL::Vector( current[0], current[1], current[2] ) );
		KDL::Frame f2( KDL::Rotation::Quaternion( cartCmd[4], cartCmd[5], cartCmd[6], cartCmd[3] ), KDL::Vector( cartCmd[0], cartCmd[1], cartCmd[2] ) );
		
		KDL::Twist twist = KDL::diff( f1, f2 );
		Eigen::Matrix<double, 6, 1> delta;
		
		std::vector<double>& K = *pK;
		std::vector<double>& D = *pD;
		
		delta << twist.vel(0) * K[0] + ( cartVelCmd.vel(0) - v[0] ) *D[0], 
				 twist.vel(1) * K[1] + ( cartVelCmd.vel(1) - v[1] ) *D[1], 
				 twist.vel(2) * K[2] + ( cartVelCmd.vel(2) - v[2] ) *D[2], 
				 twist.rot(0) * K[3] + ( cartVelCmd.rot(0) - v[3] ) *D[3], 
				 twist.rot(1) * K[4] + ( cartVelCmd.rot(1) - v[4] ) *D[4], 
				 twist.rot(2) * K[5] + ( cartVelCmd.rot(2) - v[5] ) *D[5];
		
		result.data = J.data.transpose()*delta;
		
		for( int x=0; x<jnt_size; ++x ){
			full_result( chain2Tree[x] ) = result( x );
		}
		return full_result;
	}
	
	Eigen::Matrix<double,7,1> fk( const std::vector<double>& treeJnts ){
		for( int x=0; x<jnt_size; ++x )
			jnts(x) = treeJnts[ chain2Tree[x] ];
		return fk();
	}
	
	Eigen::Matrix<double,7, 1> fk(){
		Eigen::Matrix<double,7,1> result;
			
		KDL::ChainFkSolverPos_recursive solver(chain);
		KDL::Frame frame;
		solver.JntToCart( jnts, frame );
		
		result[0] = frame.p(0);
		result[1] = frame.p(1);
		result[2] = frame.p(2);
		double& w = result[3];
		double& x = result[4];
		double& y = result[5];
		double& z = result[6];
		frame.M.GetQuaternion( x, y, z, w );
		
		return result;
	}
	Eigen::Matrix<double,6, 1> fk_vel(){
		///trouble when jnts are not between -pi, pi?
		Eigen::Matrix<double,6,1> result;
		
		KDL::ChainFkSolverVel_recursive solver(chain);
		KDL::FrameVel frame;
		
		solver.JntToCart( jntsVel, frame );
		result[0] = frame.p.v(0);
		result[1] = frame.p.v(1);
		result[2] = frame.p.v(2);
		result[3] = frame.M.w(0);
		result[4] = frame.M.w(1);
		result[5] = frame.M.w(2);
		for( int x=0; x<6; ++x ){
			assert( result[x] == result[x] );
		}
		return result;
	}
		
		
	
	
};


