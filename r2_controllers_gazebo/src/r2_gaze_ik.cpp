#include "r2_gaze_ik.hpp"

// KDL stuff
#include <kdl/frames.hpp>

// Eigen stuff
#include <Eigen/SVD>

// Standard libraries
#include <cmath>
#include <iostream>

namespace r2_gaze_controller 
{

/*template <typename Derived>
bool pinv(const Eigen::MatrixBase<Derived>& a, Eigen::MatrixBase<Derived> const & a_pinv)
{
	typedef typename Derived::Scalar Scalar;
	typedef typename Eigen::JacobiSVD<Derived>::SingularValuesType SingularValuesType;
	typedef typename Eigen::JacobiSVD<Derived>::MatrixUType MatrixUType;

    // see : http://en.wikipedia.org/wiki/Moore-Penrose_pseudoinverse#The_general_case_and_the_SVD_method
	Eigen::MatrixBase<Derived> m(a);
	Eigen::MatrixBase<Derived> m_pinv(a);

    // transpose so SVD decomp can work...
    //if (a.rows() < a.cols())
    //{
    //	m.resize(a.cols(),a.rows());
    //	m = a.transpose();
    //	m_pinv.resize(a.cols(),a.rows());
    //}
    std::cout << "Got to here." << std::endl;
    // SVD
    Eigen::JacobiSVD<Derived> svd = m.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    std::cout << "Got to here2." << std::endl;
    const SingularValuesType& vSingular = svd.singularValues();
    // Build a diagonal matrix with the Inverted Singular values
    // The pseudo inverted singular matrix is easy to compute :
    // is formed by replacing every nonzero entry by its reciprocal (inversing).
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> vPseudoInvertedSingular(svd.matrixV().cols(),1);
    for (int iRow =0; iRow < vSingular.rows(); iRow++)
    {
        if ( fabs(vSingular(iRow))<=1e-10 ) { // Todo : Put epsilon in parameter
            vPseudoInvertedSingular(iRow,0)=0.;
        } else {
            vPseudoInvertedSingular(iRow,0)=1./vSingular(iRow);
        }
    }

    // A little optimization here 
    //MatrixUType mAdjointU = svd.matrixU().adjoint().block(0,0,vSingular.rows(),svd.matrixU().adjoint().cols());
    // Pseudo-Inversion : V * S * U'
    //m_pinv = (svd.matrixV() *  vPseudoInvertedSingular.asDiagonal()) * mAdjointU;
    m_pinv = (svd.matrixV() *  vPseudoInvertedSingular.asDiagonal()) * svd.matrixU().transpose();

    // transpose back if necessary
    //if (a.rows() < a.cols())
    //{
    //	const_cast< Eigen::MatrixBase<Derived>& >(a_pinv) = m_pinv.transpose();
    //} else {
    	const_cast< Eigen::MatrixBase<Derived>& >(a_pinv) = m_pinv;
    //}

    return true;
}*/

template<typename _Matrix_Type_>
bool pseudoInverse(const _Matrix_Type_ &a, _Matrix_Type_ &result, double
		epsilon = std::numeric_limits<typename _Matrix_Type_::Scalar>::epsilon())
{
	if(a.rows()<a.cols())
		return false;

	Eigen::JacobiSVD< _Matrix_Type_ > svd = a.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

	typename _Matrix_Type_::Scalar tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs().maxCoeff();

	result = svd.matrixV() * _Matrix_Type_( (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0) ).asDiagonal() * svd.matrixU().adjoint();
	return true;
}

struct Convert
{
    static void toEigenVector(const KDL::Frame& in, Eigen::Vector3d& out)
    {
        for (int r=0; r < 3; r++) out(r) = in.p.data[r];
    }
    
    static void toJntArray(const Eigen::VectorXd& in, KDL::JntArray& out)
    {
        out.data = in;
    }
};

R2GazeIK::R2GazeIK(const KDL::Chain& chain) :
    chain(chain),
    fk(chain),
    jacobian_solver(chain),
    N(chain.getNrOfJoints()),
    Qinv(Eigen::VectorXd::Ones(N)),
    maxSolverAttempts(200),
    dt(0.1),
    p_d(Eigen::Vector3d::Zero()),
    p(Eigen::Vector3d::Zero()),
    e(Eigen::Vector3d::Zero()),
    J_partial(Jacobian::Zero(3,N)),
    Jw(Jacobian::Zero(3,N)),
    Jwt(Jacobian::Zero(N,3)),
    Jwt_inv(Jacobian::Zero(N,3)),
    q_kdl(N),
    J_kdl(N),
    del_q(N), q_local(N)
{
}

R2GazeIK::~R2GazeIK()
{

}

void R2GazeIK::setWeightMatrix(const Eigen::VectorXd& w)
{
    Qinv.diagonal() = w;
}

void R2GazeIK::setWeightMatrix(const Eigen::MatrixXd& W)
{
	Eigen::VectorXd w(W.diagonal());
	this->setWeightMatrix(w);
}

double R2GazeIK::computeDelta(const Eigen::VectorXd& q, const KDL::Frame& desired_frame, Eigen::VectorXd& del_q)
{
    // Get desired position to look at
    Convert::toEigenVector(desired_frame, p_d);

    // Create a joint array from q from actual neck angles
    Convert::toJntArray(q, q_kdl);

    // Use forward kinematics to determine where we are currently at
    KDL::Frame frame_actual;
    if (fk.JntToCart(q_kdl, frame_actual) < 0)
    {
        return -1;
    }
    Convert::toEigenVector(frame_actual, p);
                    
    // Determine error - where we are currently and where we want to be
    e = p_d - p;

    // Compute chain jacobian at q
    jacobian_solver.JntToJac(q_kdl, J_kdl);

    // Only interested in the position submatrix of J
    J_partial = J_kdl.data.topRows(3);

    // Weighted jacobian and its inverse
    Jw = J_partial * Qinv * J_partial.transpose();
    Jwt = Jw.transpose();
    pseudoInverse(Jwt, Jwt_inv);

    // Compute delta_q step
    del_q = Qinv * J_partial.transpose() * Jwt_inv.transpose() * e;
    return e.transpose() * e;
}

int R2GazeIK::computeSolution(const KDL::JntArray& q_act, const KDL::Frame& desired_frame, KDL::JntArray& q)
{
	Eigen::VectorXd q_in(q_act.data), q_out(q_act.data);
	int ret = computeSolution(q_in, desired_frame, q_out);
	q.data = q_out;
	return ret;
}

int R2GazeIK::computeSolution(const Eigen::VectorXd& q_act, const KDL::Frame& desired_frame, Eigen::VectorXd& q)
{
    // Out starting point
    q_local = q_act;

    // Find out where we are at
    double err = computeDelta(q_local, desired_frame, del_q);

    if (err < 0) return -1;
    
    // Iterate until position error is small
    int attempts = 0;
    while ((err > 0.01) & (attempts < maxSolverAttempts))
    {
        q_local += dt * del_q;
        err = computeDelta(q_local, desired_frame, del_q);
        if (err < 0) return -1;
        attempts = attempts + 1;
    } 

    q = q_local;
                 
    if (attempts >= maxSolverAttempts) return -1;
    return 0;
}
   
} // end namespace
