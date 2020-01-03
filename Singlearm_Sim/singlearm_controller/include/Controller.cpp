/*
 * Controller.cpp
 *
 *  Created on: 2019. 5. 15.
 *      Author: Administrator
 */

#include "Controller.h"

namespace HYUControl {

Controller::Controller():m_Jnum(6)
{
	this->pManipulator = NULL;
	m_KpBase = KpBase;
	m_KdBase = KdBase;
	m_HinfBase = HinfBase;
}

Controller::Controller(SerialManipulator *pManipulator)
{
	this->pManipulator = pManipulator;

	m_Jnum = pManipulator->GetTotalDoF();
	m_KpBase = KpBase;
	m_KdBase = KdBase;
	m_HinfBase = HinfBase;

	Kp.resize(m_Jnum);
	Kp.setConstant(m_KpBase);
	Kd.resize(m_Jnum);
	Kd.setConstant(m_KdBase);

	K_Hinf.resize(m_Jnum);
	K_Hinf.setConstant(m_HinfBase);

	KpTask.resize(6*pManipulator->pKin->GetNumChain());
	KdTask.resize(6*pManipulator->pKin->GetNumChain());
	KiTask.resize(6*pManipulator->pKin->GetNumChain());

	e.resize(m_Jnum);
	e.setZero();

	e_dev.resize(m_Jnum);
	e_dev.setZero();

	e_int.resize(m_Jnum);
	e_int_sat.resize(m_Jnum);

	edotTask.resize(6*pManipulator->GetTotalChain());
	eTask.resize(6*pManipulator->GetTotalChain());

	edotTmp.resize(6*pManipulator->GetTotalChain(), 6*pManipulator->GetTotalChain());
	dexp.resize(6,6);

	ToqOut.resize(m_Jnum);
	ToqOut.setZero();

	FrictionTorque.resize(m_Jnum);
	FrictionTorque.setZero();

	GainWeightFactor.resize(m_Jnum);

	GainWeightFactor(0) = 21.0;
	GainWeightFactor(1) = 21.0;

	GainWeightFactor(2) = 10.5;
	GainWeightFactor(3) = 10.5; //8
	GainWeightFactor(4) = 8.0;
	GainWeightFactor(5) = 4.0;
	GainWeightFactor(6) = 3.5;
	GainWeightFactor(7) = 3.5;

	GainWeightFactor(8) = 10.5;
	GainWeightFactor(9) = 10.5; //8
	GainWeightFactor(10) = 8.0;
	GainWeightFactor(11) = 4.0;
	GainWeightFactor(12) = 3.7;
	GainWeightFactor(13) = 3.5;


	Kp = GainWeightFactor*m_KpBase;
	Kd = GainWeightFactor*m_KdBase;
	//K_Hinf = m_HinfBase;

	dq.resize(m_Jnum);
	dqdot.resize(m_Jnum);
	dqddot.resize(m_Jnum);

	KpTask(0) = 0.00001;
	KpTask(1) = 0.00001;
	KpTask(2) = 0.00001;

	KpTask(3) = 0.0001;
	KpTask(4) = 0.0001;
	KpTask(5) = 0.0001;

	KpTask.tail(6) = KpTask.head(6);

}

Controller::~Controller() {

}


void Controller::ClearError(void)
{
	e.setZero();
	e_dev.setZero();
	e_int.setZero();
	return;
}

void Controller::SetPIDGain(double &_Kp, double &_Kd, double &_Hinf, int &_JointNum)
{
	Kp(_JointNum-1) = _Kp;
	Kd(_JointNum-1) = _Kd;
	K_Hinf(_JointNum-1) = _Hinf;
	return;
}

void Controller::SetPIDGain(Eigen::VectorXd & _Kp, Eigen::VectorXd & _Kd, Eigen::VectorXd & _Ki)
{
    Kp = _Kp;
    Kd = _Kd;
    K_Hinf = _Ki;
    return;
}

void Controller::GetPIDGain(double *_Kp, double *_Kd, double *_Hinf, int &_JointNum)
{
	_JointNum = this->m_Jnum;
	Map<VectorXd>(_Kp, this->m_Jnum) = Kp;
	Map<VectorXd>(_Kd, this->m_Jnum) = Kd;
	Map<VectorXd>(_Hinf, this->m_Jnum) = K_Hinf;
	return;
}

void Controller::GetPIDGain(Eigen::VectorXd & _Kp, Eigen::VectorXd & _Kd, Eigen::VectorXd & _Ki)
{
    _Kp = Kp;
    _Kd = Kd;
    _Ki = K_Hinf;
}

void Controller::PDController(double *p_q, double *p_qdot, double *p_dq, double *p_dqdot, double *p_Toq, double &_dt)
{
	q = Map<VectorXd>(p_q, this->m_Jnum);
	qdot = Map<VectorXd>(p_qdot, this->m_Jnum);

	dq = Map<VectorXd>(p_dq, this->m_Jnum);
	dqdot = Map<VectorXd>(p_dqdot, this->m_Jnum);

	e = dq - q;
	e_dev = dqdot - qdot;

	ToqOut.setZero();
	ToqOut.noalias() += e.cwiseProduct(Kp) + e_dev.cwiseProduct(Kd);

	Map<VectorXd>(p_Toq, this->m_Jnum) = ToqOut;
	return;
}

void Controller::PDGravController( double *p_q, double *p_qdot, double *p_dq, double *p_dqdot, double *p_Toq, double &_dt )
{
	pManipulator->pDyn->G_Matrix(G);

	q = Map<VectorXd>(p_q, this->m_Jnum);
	qdot = Map<VectorXd>(p_qdot, this->m_Jnum);

	dq = Map<VectorXd>(p_dq, this->m_Jnum);
	dqdot = Map<VectorXd>(p_dqdot, this->m_Jnum);

	e = dq - q;
	e_dev = dqdot - qdot;

	FrictionCompensator(qdot, dqdot);

	ToqOut.setZero();
	ToqOut = Kp.cwiseProduct(e) + Kd.cwiseProduct(e_dev) + G;
    //ToqOut = Kp.cwiseProduct(e) + Kd.cwiseProduct(e_dev) ;
	//ToqOut = G + FrictionTorque;

	Map<VectorXd>(p_Toq, this->m_Jnum) = ToqOut;
	return;
}

void Controller::InvDynController( VectorXd &_q, VectorXd &_qdot, VectorXd &_dq, VectorXd &_dqdot, VectorXd &_dqddot, double *p_Toq, double &_dt )
{
	pManipulator->pDyn->MG_Mat_Joint(M, G);

	e = _dq - _q;
	e_dev = _dqdot - _qdot;
	//e_int += e*1e-3;
	//e_int_sat = tanh(e_int);

	//FrictionCompensator(qdot, dqdot);

	ToqOut.setZero();
    ToqOut =  K_Hinf.cwiseProduct( Kd.cwiseProduct(e_dev) + Kp.cwiseProduct(e)) + G ;
    //ToqOut =  M*( dqddot + K_Hinf.cwiseProduct( Kd.cwiseProduct(e_dev) + Kp.cwiseProduct(e)) )+ G ;
	Map<VectorXd>(p_Toq, this->m_Jnum) = ToqOut;
	return;

}

void Controller::CLIKTaskController( double *_q, double *_qdot, double *_dq, double *_dqdot, const VectorXd *_dx, const VectorXd *_dxdot, const VectorXd &_dqdotNull, double *p_Toq, double &_dt )
{
	pManipulator->pDyn->G_Matrix(G);

	q = Map<VectorXd>(_q, this->m_Jnum);
	qdot = Map<VectorXd>(_qdot, this->m_Jnum);

	pManipulator->pKin->GetScaledTransJacobian(this->ScaledTransJacobian);
	pManipulator->pKin->GetpinvJacobian(this->pInvJacobian);
	pManipulator->pKin->GetBodyJacobian(this->BodyJacobian);

	eTask.setZero();
	edotTmp.setZero();
	edotTask.setZero();

	for(int i=0; i<pManipulator->GetTotalChain(); i++)
	{
		dSE3.block<3,3>(0,0) = LieOperator::ExpSO3Matrix(_dx[i].head(3), _dx[i](3));
		dSE3.block<3,1>(0,3) = _dx[i].segment(4,3);

		eSE3 = LieOperator::inverse_SE3(pManipulator->pKin->GetForwardKinematicsSE3(i))*dSE3;

		LogSO3(eSE3.block(0,0,3,3), omega, theta);

		eTask.segment(6*i,3) = omega*theta;
		eTask.segment(6*i+3,3) = eSE3.block(0,3,3,1);

		edotTask.segment(6*i, 6) = _dxdot[i];
	}

	dq.setZero();
	dqdot.setZero();
	dqddot.setZero();

	dqdot = ScaledTransJacobian*( edotTask + KpTask.cwiseProduct(eTask) );
	dq = q + dqdot*_dt;

	e = dq - q;
	e_dev = dqdot - qdot;

	//FrictionCompensator(qdot, dqdot);

	ToqOut.setZero();
	ToqOut = K_Hinf.cwiseProduct(Kd.cwiseProduct(e_dev) + Kp.cwiseProduct(e)) + G;

	Map<VectorXd>(_dq, this->m_Jnum) = dq;
	Map<VectorXd>(_dqdot, this->m_Jnum) = dqdot;
	Map<VectorXd>(p_Toq, this->m_Jnum) = ToqOut;

	return;
}

void Controller::FrictionIdentification( double *p_q, double *p_qdot, double *p_dq, double *p_dqdot, double *p_dqddot, double *p_Toq, double &gt )
{
	pManipulator->pDyn->MG_Mat_Joint(M, G);

	q = Map<VectorXd>(p_q, this->m_Jnum);
	qdot = Map<VectorXd>(p_qdot, this->m_Jnum);

	GainWeightFactor(0) = 7.0;
	GainWeightFactor(1) = 4.0;

	GainWeightFactor(2) = 2.0;
	GainWeightFactor(3) = 1.5; //8
	GainWeightFactor(4) = 3.0;
	GainWeightFactor(5) = 1.0;
	GainWeightFactor(6) = 0.8;
	GainWeightFactor(7) = 1.0;

	GainWeightFactor(8) = 2.0;
	GainWeightFactor(9) = 1.5; //8
	GainWeightFactor(10) = 3.0;
	GainWeightFactor(11) = 1.0;
	GainWeightFactor(12) = 0.8;
	GainWeightFactor(13) = 1.0;


	Kp = GainWeightFactor*m_KpBase;
	Kd = GainWeightFactor*m_KdBase;

	dq.setZero();
	dqdot.setZero();
	dqddot.setZero();

	int testjoint = 4;

	double T, omega, amp;

	if(InitTime == 0)
	{
		InitTime = gt;
	}
	else
	{
		switch(testjoint)
		{
		case 0:
			T = 29.3;
			omega = 2.0*M_PI/T;
			amp = 70;

			dq(testjoint) = amp*M_PI/180.0*sin(omega*(gt-InitTime));
			dqdot(testjoint) = amp*M_PI/180.0*omega*cos(omega*(gt-InitTime));
			dqddot(testjoint) = -amp*M_PI/180*pow(omega,2)*sin(omega*(gt-InitTime));
			break;

		case 1:
			T = 16.7;
			omega = 2.0*M_PI/T;
			amp = 40;

			dq(testjoint) = amp*M_PI/180.0*sin(omega*(gt-InitTime)+0.8481) - 30*M_PI/180.0;
			dqdot(testjoint) = amp*M_PI/180.0*omega*cos(omega*(gt-InitTime)+0.8481);
			dqddot(testjoint) = -amp*M_PI/180*pow(omega,2)*sin(omega*(gt-InitTime)+0.8481);
			break;

		case 2:
			T = 20.9;
			omega = 2.0*M_PI/T;
			amp = 50;

			dq(testjoint) = amp*M_PI/180.0*sin(omega*(gt-InitTime)+0.6435) - 30*M_PI/180.0;
			dqdot(testjoint) = amp*M_PI/180.0*omega*cos(omega*(gt-InitTime)+0.6435);
			dqddot(testjoint) = -amp*M_PI/180*pow(omega,2)*sin(omega*(gt-InitTime)+0.6435);

			dq(testjoint+6) = -dq(testjoint);
			dqdot(testjoint+6) = -dqdot(testjoint);
			dqddot(testjoint+6) = -dqddot(testjoint);
			break;

		case 3:
			T = 30.0;
			omega = 2.0*M_PI/T;
			amp = 50;

			dq(testjoint) = amp*M_PI/180.0*sin(omega*(gt-InitTime)+0.4115) - 20*M_PI/180.0;
			dqdot(testjoint) = amp*M_PI/180.0*omega*cos(omega*(gt-InitTime)+0.4115);
			dqddot(testjoint) = -amp*M_PI/180*pow(omega,2)*sin(omega*(gt-InitTime)+0.4115);

			dq(testjoint+6) = -dq(testjoint);
			dqdot(testjoint+6) = -dqdot(testjoint);
			dqddot(testjoint+6) = -dqddot(testjoint);
			break;

		case 4:
			T = 18.8;
			omega = 2.0*M_PI/T;
			amp = 45;

			dq(testjoint) = amp*M_PI/180.0*sin(omega*(gt-InitTime)+0.729) - 30*M_PI/180.0;
			dqdot(testjoint) = amp*M_PI/180.0*omega*cos(omega*(gt-InitTime)+0.729);
			dqddot(testjoint) = -amp*M_PI/180*pow(omega,2)*sin(omega*(gt-InitTime)+0.729);

			dq(testjoint+6) = -dq(testjoint);
			dqdot(testjoint+6) = -dqdot(testjoint);
			dqddot(testjoint+6) = -dqddot(testjoint);
			break;

		case 5:
			T = 29.3;
			omega = 2.0*M_PI/T;
			amp = 70.0;

			dq(testjoint) = amp*M_PI/180.0*sin(omega*(gt-InitTime));
			dqdot(testjoint) = amp*M_PI/180.0*omega*cos(omega*(gt-InitTime));
			dqddot(testjoint) = -amp*M_PI/180*pow(omega,2)*sin(omega*(gt-InitTime));

			dq(testjoint+6) = dq(testjoint);
			dqdot(testjoint+6) = dqdot(testjoint);
			dqddot(testjoint+6) = dqddot(testjoint);
			break;

		case 6:
			T = 16.7;
			omega = 2.0*M_PI/T;
			amp = 40.0;

			dq(testjoint) = amp*M_PI/180.0*sin(omega*(gt-InitTime));
			dqdot(testjoint) = amp*M_PI/180.0*omega*cos(omega*(gt-InitTime));
			dqddot(testjoint) = -amp*M_PI/180*pow(omega,2)*sin(omega*(gt-InitTime));

			dq(testjoint+6) = dq(testjoint);
			dqdot(testjoint+6) = dqdot(testjoint);
			dqddot(testjoint+6) = dqddot(testjoint);
			break;

		case 7:
			T = 16.7;
			omega = 2.0*M_PI/T;
			amp = 40.0;

			dq(testjoint) = amp*M_PI/180.0*sin(omega*(gt-InitTime));
			dqdot(testjoint) = amp*M_PI/180.0*omega*cos(omega*(gt-InitTime));
			dqddot(testjoint) = -amp*M_PI/180*pow(omega,2)*sin(omega*(gt-InitTime));

			dq(testjoint+6) = dq(testjoint);
			dqdot(testjoint+6) = dqdot(testjoint);
			dqddot(testjoint+6) = dqddot(testjoint);
			break;

		default:
			dq.setZero();
			dqdot.setZero();
			dqddot.setZero();
			break;
		}
	}

	e = dq - q;
	e_dev = dqdot - qdot;

	FrictionCompensator(qdot, dqdot);

	//ToqOut = M.diagonal().cwiseProduct(dqddot) + Kp.cwiseProduct(e) + Kd.cwiseProduct(e_dev) + G + FrictionTorque;
	//ToqOut = M.diagonal().cwiseProduct(dqddot) + Kp.cwiseProduct(e) + Kd.cwiseProduct(e_dev) + G;
	ToqOut = Kp.cwiseProduct(e) + Kd.cwiseProduct(e_dev) + G;

	Map<VectorXd>(p_dq, this->m_Jnum) = dq;
	Map<VectorXd>(p_dqdot, this->m_Jnum) = dqdot;
	Map<VectorXd>(p_dqddot, this->m_Jnum) = dqddot;
	Map<VectorXd>(p_Toq, this->m_Jnum) = ToqOut;
	return;

}

void Controller::FrictionCompensator( VectorXd &_qdot, VectorXd &_dqdot )
{
	FrictionTorque.setZero();

	for(int i=0; i < this->m_Jnum; i++)
	{
		FrictionTorque(i) = frictiontanh[i].a*(tanh(frictiontanh[i].b*_dqdot(i)) - tanh(frictiontanh[i].c*_dqdot(i))) + frictiontanh[i].d*tanh(frictiontanh[i].e*_dqdot(i)) + frictiontanh[i].f*_dqdot(i);
	}
	return;
}

void Controller::OutputSaturation(double *pInput , double &_MaxInput)
{
	for(int i=0; i<m_Jnum; ++i)
	{
		if(pInput[i] <= -_MaxInput)
		{
			pInput[i] = -_MaxInput;
		}
		else if(pInput[i] >= _MaxInput)
		{
			pInput[i] = _MaxInput;
		}
	}
	return;
}


} /* namespace HYUCtrl */
