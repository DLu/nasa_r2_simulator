#ifndef _R2_GAZE_IK_HPP_
#define _R2_GAZE_IK_HPP_

// KDL stuff
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

// Eigen stuff
#include <Eigen/Core>

namespace r2_gaze_controller
{

class R2GazeIK
{
    typedef Eigen::DiagonalMatrix<double,Eigen::Dynamic> DiagMatrix;
    typedef Eigen::MatrixXd Jacobian;
    
public:
    R2GazeIK(const KDL::Chain& chain);
    ~R2GazeIK();
    
    void setWeightMatrix(const Eigen::VectorXd& w);
    void setWeightMatrix(const Eigen::MatrixXd& W);
    int computeSolution(const Eigen::VectorXd& q_act, const KDL::Frame& desired_frame, Eigen::VectorXd& q);
    int computeSolution(const KDL::JntArray& q_act, const KDL::Frame& desired_frame, KDL::JntArray& q);
    
private:

    double computeDelta(const Eigen::VectorXd& q, const KDL::Frame& desired_frame, Eigen::VectorXd& del_q);

    // KDL chain, forward kinematics and a jacobian solver
    KDL::Chain chain;
    KDL::ChainFkSolverPos_recursive fk;
    KDL::ChainJntToJacSolver jacobian_solver;
    
    // Stores the number of degrees of freedom in chain
    int N;
    
    // Weight matrix and its inverse
    DiagMatrix Q;
    DiagMatrix Qinv;

    // Parameters which control the ik solver
    int maxSolverAttempts;
    double dt;
    
    // Temporaries
    Eigen::Vector3d p_d, p, e;
    Jacobian J_partial, Jw, Jwt, Jwt_inv;
    Eigen::VectorXd del_q, q_local;
    KDL::JntArray q_kdl;
    KDL::Jacobian J_kdl;
};

}

#endif

