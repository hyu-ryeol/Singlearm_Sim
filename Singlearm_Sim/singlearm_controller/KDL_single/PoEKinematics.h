/**
 * @file PoEKinematics.h
 * @brief Product of Exponential formulation for Kinematics
 * @date 2019-09-17
 * @author Junho Park
 */

#ifndef POEKINEMATICS_H_
#define POEKINEMATICS_H_

#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>

#include <Eigen/Dense>
using namespace Eigen;

#include "LieOperator.h"
#include "PropertyDefinition.h"

/**
 * @brief [Biorobotics Lab] Kinematics Solver using Lie-Group(Differential Kinematics)
 * @version 1.2.0
 */
namespace HYUMotionKinematics {

/**
 * @brief PoEKinematics Class for Tree-type Manipulator
 * @version 1.2.0
 */
class PoEKinematics : public HYUMotionBase::LieOperator {
public:

	/**
	 * @brief PoEKinematics class constructor
	 * @details A chain matrix should be defined.
	 */
	PoEKinematics();
	/**
	 * @brief PoEKinematics class constructor
	 * @details A chain matrix should be defined.
	 */
	PoEKinematics( const MatrixXi &_ChainMat );
	virtual ~PoEKinematics();
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	/**
	 * @brief Construct the kinematic infomation
	 * @param[in] _w omega(twist)
	 * @param[in] _p link position
	 * @param[in] _l link length
	 * @param[in] _link_num number of link attached to base-coordinate
	 */
	void UpdateKinematicInfo( Vector3d _w, Vector3d _p, Vector3d _l, int _link_num );

	/**
	 * @brief Calculate the joint velocity v
	 * @param[in] _w joint axis with respect to the base coordinate
	 * @param[in] _p lint position attached to joint coordinate
	 * @return v
	 */
	Vector3d GetV( const Vector3d &_w, const Vector3d &_p );

	/**
	 * @brief Calculate the initial configuration of serial robot
	 * @param[in] _link total length of robot
	 * @return SE(3)
	 */
	SE3 GetM( const Vector3d &_link );

	/**
	 * @brief Calculate the Twist of joint
	 * @param[in] _w joint axis with respect to the base coordinate
	 * @param[in] _v joint velocity
	 * @return se3 vector
	 */
	se3 GetTwist( const Vector3d &_w, const Vector3d &_v );

	/**
	 * @brief Calculate the Homogeneous transformation matrix SE(3)
	 * @param[in] _q generalized coordinate of joint position
	 */
	void HTransMatrix( const double *_q );

	void PrepareJacobian( const double *_q );
	/**
	 * @brief calculate the space jacobian
	 * @return 6 x n(DoF) jacobian matrix w.r.t, base coordinate
	 */
	void GetSpaceJacobian( MatrixXd &_SpaceJacobian )
	{
		_SpaceJacobian = mSpaceJacobian;
		return;
	}

	/**
	 * @brief calculate the body jacobian
	 * @return 6 x n(DoF) jacobian matrix w.r.t., end-effector coordinate
	 */
	void GetBodyJacobian( MatrixXd &_BodyJacobian )
	{
		_BodyJacobian = mBodyJacobian;
		return;
	}

	/**
	 * @brief calcuate the analytic jacobian
	 * @return 6 x n(DoF) jacobian matrix
	 */
	void GetAnalyticJacobian( MatrixXd &_AnalyticJacobian )
	{
		_AnalyticJacobian = mAnalyticJacobian;
		return;
	}

	void GetpinvJacobian( MatrixXd &_pinvJacobian );

	void GetScaledTransJacobian( MatrixXd &_ScaledTransJacobian );

	void GetTaskVelocity( double *_qdot, VectorXd *_TaskVelocity, int &_size );

	void GetManipulability( double *_TaskEigen, double *_OrientEigen );

	double GetManipulabilityMeasure(void);
	/**
	 * @brief forward kinematics of serial robot
	 * @return end-effector position x, y, z. not orientation(Working)
	 */
	void GetForwardKinematics( Vector3d *_Position, Vector3d *_Orientation, int &_NumChain );

	SE3 GetForwardKinematicsSE3( const int &_EndPosition ) const;

	void GetAngleAxis( Vector3d *_Axis, double *_Angle, int &_NumChain );

	void SO3toRollPitchYaw( const Matrix3d &_RotMat, Vector3d &_Orientation );

	void RollPitchYawtoSO3( const double &_Roll_rad, const double &_Pitch_rad, const double &_Yaw_rad, Matrix3d &_RotMat);

	SE3 GetTMat(int _begin, int _end)
	{
		return T[_begin][_end];
	}

	int GetNumChain(void) const
	{
		return m_NumChain;
	}

	se3 GetTwist(int _pos) const
	{
		return v_se3[_pos];
	}

	SE3 GetMMat(int _pos) const
	{
		return M[_pos];
	}

private:

	void SpaceJacobian(void);

	void SpaceToBodyJacobian(void);

	void AnalyticJacobian(void);

	void ScaledTransJacobian(void);

	MatrixXd mSpaceJacobian;
	MatrixXd mBodyJacobian;
	MatrixXd mAnalyticJacobian;
	MatrixXd mRelativeJacobian;

	MatrixXi ChainMatrix;

	int m_NumChain;
	int m_DoF;

	int ChainJointCount[2];
	int JointEndNum[2];

	SE3 SE3_Tmp;
	se3 se3_Tmp;

	VectorXi Arr[2];

	VectorXd ScaledFactor;
	MatrixXd mScaledTransJacobian;

	MatrixXd Mat_Tmp;
	VectorXd Vec_Tmp;

	Quaterniond q;

	Vector3d Omega;
	Vector3d r;
	double Theta;

	/**
	 * @brief SE(3) Homogeneous transform matrix container
	 */
	SE3 T[16][16];
	//SE3 **T;

	/**
	 * @brief SE(3) matrix container w.r.t., base coordinate
	 */
	SE3 M[16];
	//SE3 *M;

	/**
	 * @brief SE(3) matrix container
	 */
	SE3 Exp_S[16];
	//SE3 *Exp_S;

	/**
	 * @brief twist expression for Adjoint/adjoint matrix
	 */
	se3 v_se3[16];
	//se3 *v_se3;

	/**
	 * @brief Kinematic infomation update flag
	 */
	volatile int isInfoUpdated;

};

}

#endif /* POEKINEMATICS_H_ */
