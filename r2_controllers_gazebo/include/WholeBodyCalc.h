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

/// performs the successive nullspace calculations for the whole body framework
class WholeBodyCalc{
	Eigen::MatrixXd Jsum;
	Eigen::MatrixXd I;
	Eigen::MatrixXd N;
	KDL::JntArray result;
	
	int tree_size;
	
	public:
	WholeBodyCalc():tree_size(0){}
	WholeBodyCalc(const KDL::Tree& tree){
		tree_size = tree.getNrOfJoints();
		Jsum.resize( tree_size, tree_size );
		Jsum = Eigen::MatrixXd::Zero(tree_size, tree_size );
		I.resize( tree_size, tree_size );
		I.setIdentity();
		N.resize( tree_size, tree_size );
		N.setIdentity();
		result.resize( tree_size );
	}
	
	void reset(){
		N.setIdentity();
		Jsum = Eigen::MatrixXd::Zero(tree_size, tree_size );
	}
	
	/// project torques through the current nullspace, add the nullspace of tc for the next call
	const KDL::JntArray& project( const KDL::JntArray& torques, const TreeChain& tc ){
		result.data = N * torques.data;
		Eigen::MatrixXd JN = tc.getJ().data*N;
		Eigen::MatrixXd JNinv = calcPinv(JN);
		Jsum += JNinv * JN;
		N = I - Jsum.transpose();  // Jsum.transpose()?...Jsum should be symmetrical, maybe
		
		return result;
	}
	/// project torques through the current nullspace but leave the nullspace the same
	const KDL::JntArray& project( const KDL::JntArray& torques ){
		result.data = N * torques.data;
		return result;
	}
	Eigen::MatrixXd calcPinv( const Eigen::MatrixXd& in );
	
};

