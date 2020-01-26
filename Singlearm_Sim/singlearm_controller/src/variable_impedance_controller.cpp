
#include <control_msgs/JointControllerState.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <angles/angles.h>
#include <geometry_msgs/WrenchStamped.h>
#include "singlearm_controller/ControllerJointState.h"
#include "singlearm_controller/ControllerMatlabState.h"

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>

#include <boost/scoped_ptr.hpp>

#define PI 3.141592
#define D2R PI/180.0
#define R2D 180.0/PI
#define JointMax 6
#define SaveDataMax 7

namespace singlearm_controller{

class VariableImpedanceController: public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
  ~VariableImpedanceController() override
  {
    sub_command_.shutdown();
    sub_forcetorque_sensor_.shutdown();
  }

  bool init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n) override
  {
    ROS_INFO("Initialize Variable Impedance Controller");
    // List of controlled joints
    if (!n.getParam("joints", joint_names_))
    {
      ROS_ERROR("Could not find joint name");
      return false;
    }

    n_joints_ = joint_names_.size();

    if(n_joints_ == 0)
    {
      ROS_ERROR("List of joint names is empty.");
      return false;
    }
    ROS_INFO("Initialize URDF Model");
    // urdf
    urdf::Model urdf;
    if (!urdf.initParam("singlearm/robot_description"))
    {
      ROS_ERROR("Failed to parse urdf file");
      return false;
    }

    // joint handle
    for(int i=0; i<n_joints_; i++)
    {
      try
      {
        joints_.push_back(hw->getHandle(joint_names_[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }

      urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
      if (!joint_urdf)
      {
        ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
        return false;
      }
      joint_urdfs_.push_back(joint_urdf);
    }
    ROS_INFO("Ready for KDL solver");
    // kdl parser
    if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
    }

    // kdl chain
    std::string root_name, tip_name;
    if (!n.getParam("root_link", root_name))
    {
      ROS_ERROR("Could not find root link name");
      return false;
    }
    if (!n.getParam("tip_link", tip_name))
    {
      ROS_ERROR("Could not find tip link name");
      return false;
    }
    if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
    {
      ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
      ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
      ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
      ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
      ROS_ERROR_STREAM("  The segments are:");

      KDL::SegmentMap segment_map = kdl_tree_.getSegments();
      KDL::SegmentMap::iterator it;

      for( it=segment_map.begin(); it != segment_map.end(); it++ )
        ROS_ERROR_STREAM( "    "<<(*it).first);

      return false;
    }

    gravity_ = KDL::Vector::Zero();
    gravity_(2) = -9.81;

    // inverse dynamics solver
    id_solver_.reset( new KDL::ChainDynParam(kdl_chain_, gravity_) );
    fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    //ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_, fk_solver_, ik_vel_solver_));

    // command and state
    tau_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
    d_.data = Eigen::VectorXd::Zero(n_joints_);
    d_Cart_.data = Eigen::VectorXd::Zero(6);

    dq_.data = Eigen::VectorXd::Zero(n_joints_);
    dq_dot_.data = Eigen::VectorXd::Zero(n_joints_);
    dq_dot_old_.data = Eigen::VectorXd::Zero(n_joints_);
    dq_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
    dq_end_.data = Eigen::VectorXd::Zero(n_joints_);

    q_.data = Eigen::VectorXd::Zero(n_joints_);
    q_dot_.data = Eigen::VectorXd::Zero(n_joints_);
    q_init_.data = Eigen::VectorXd::Zero(n_joints_);

    e_.data = Eigen::VectorXd::Zero(n_joints_);
    e_dot_.data = Eigen::VectorXd::Zero(n_joints_);

    for (size_t i = 0; i < 6; i++)
    {
      Xc_dot_(i) = 0.0;
    }

    Kp_.resize(n_joints_);
    Kd_.resize(n_joints_);
    Kp_dot_.resize(n_joints_);
    Kp_dot_.setZero();

    experiment_mode_ = 0;

    std::vector<double> Kp(n_joints_), Kd(n_joints_);
    for (size_t i=0; i<n_joints_; i++)
    {
      std::string si = std::to_string(i+1);
      //Kp
      if ( n.getParam("/singlearm/variable_impedance_controller/joint" + si + "/tdc/p", Kp[i]) )
      {
        Kp_(i) = Kp[i];
      }
      else
      {
        ROS_ERROR("Cannot find Kp gain");
        return false;
      }
      //Kd
      if ( n.getParam("/singlearm/variable_impedance_controller/joint" + si + "/tdc/d", Kd[i]) )
      {
        Kd_(i) = Kd[i];
      }
      else
      {
        ROS_ERROR("Cannot find Kd gain");
        return false;
      }
    }

    if (!n.getParam("/singlearm/variable_impedance_controller/mode", experiment_mode_))
    {
      ROS_ERROR("Cannot find mode");
      return false;
    }

    if (!n.getParam("/singlearm/variable_impedance_controller/aic/alpha", alpha))
    {
      ROS_ERROR("Cannot find alpha");
      return false;
    }

    if (!n.getParam("/singlearm/variable_impedance_controller/aic/epsilon", epsilon))
    {
      ROS_ERROR("Cannot find alpha");
      return false;
    }

    J_.resize(kdl_chain_.getNrOfJoints());
    M_mat.resize(kdl_chain_.getNrOfJoints());
    M_mat_p.resize(kdl_chain_.getNrOfJoints());
    M_mat_dot.resize(kdl_chain_.getNrOfJoints());
    C_mat.resize(kdl_chain_.getNrOfJoints());
    G_mat.resize(kdl_chain_.getNrOfJoints());

    // publisher
    controller_state_pub_.reset(new realtime_tools::RealtimePublisher<singlearm_controller::ControllerMatlabState>(n, "state", 1));
    controller_state_pub_->msg_.header.stamp = ros::Time::now();
    for(size_t i=0; i<6*(n_joints_-1)+4; i++)
    {
      controller_state_pub_->msg_.data.push_back(0.0);
    }

    // subsriber
    commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
    sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command",1, &VariableImpedanceController::commandCB,this);
    sub_forcetorque_sensor_ = n.subscribe<geometry_msgs::WrenchStamped>("/singlearm/singlearm/ft_sensor_topic", 1, &VariableImpedanceController::updateFTsensor, this);

    return true;
  }

  void starting(const ros::Time& time) override
  {
    // get joint positions
    for(size_t i=0; i<n_joints_; i++)
    {
      ROS_INFO("JOINT %d", (int)i);
      q_(i) = joints_[i].getPosition();
      q_dot_(i) = joints_[i].getVelocity();
    }

    q_init_ = q_;

    id_solver_->JntToMass(q_, M_mat);
    M_mat_p = M_mat;

    time_ = time.toSec();
    total_time_ = time.toSec();

    ROS_INFO("Starting Variable Impedance Controller");
  }

  void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
  {
    if(msg->data.size()!=n_joints_)
    {
      ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
      return;
    } else{
      commands_buffer_.writeFromNonRT(msg->data);
    }
  }

  void updateFTsensor(const geometry_msgs::WrenchStamped::ConstPtr &msg)
  {
    // Convert Wrench msg to KDL wrench
    geometry_msgs::Wrench f_meas = msg->wrench;

    f_cur_(0) = f_meas.force.x;
    f_cur_(1) = f_meas.force.y;
    f_cur_(2) = f_meas.force.z;
    f_cur_(3) = f_meas.torque.x;
    f_cur_(4) = f_meas.torque.y;
    f_cur_(5) = f_meas.torque.z;

    d_Cart_(0) = f_cur_(0);
    d_Cart_(1) = f_cur_(1);
    d_Cart_(2) = f_cur_(2);
    d_Cart_(3) = f_cur_(3);
    d_Cart_(4) = f_cur_(4);
    d_Cart_(5) = f_cur_(5);
  }

  void update(const ros::Time& time, const ros::Duration& period) override
  {
    std::vector<double> &commands = *commands_buffer_.readFromRT();
    t = time;
    dt_ = period.toSec();
    total_time_ = t.toSec();

    // get joint states
    for (size_t i=0; i<n_joints_; i++)
    {
      q_(i) = joints_[i].getPosition();
      q_dot_(i) = joints_[i].getVelocity();
    }

    if(total_time_ < 4.0)
    {
      task_init();
    }
    else if(total_time_ >= 4.0 && total_time_ < 5.0)
    {
      task_via();
    }
    else if(total_time_ >= 5.0 && total_time_ < 12.0)
    {
      task_pos1();
    }
    else if (total_time_ >= 12.0 && total_time_ < 19.0)
    {
      task_pos2();
    }
    else
    {
      task_homming();
    }

    id_solver_->JntToMass(q_, M_mat);
    id_solver_->JntToCoriolis(q_, q_dot_, C_mat);
    id_solver_->JntToGravity(q_, G_mat);
    jnt_to_jac_solver_->JntToJac(q_, J_);

    M_mat_dot.data = (M_mat.data - M_mat_p.data)/dt_;

    e_.data = dq_.data - q_.data;
    e_dot_.data = dq_dot_.data - q_dot_.data;

    d_.data = J_.data.transpose()*d_Cart_.data;

    GainUpdate();

    tau_cmd_.data = M_mat.data*dq_ddot_.data + Kp_.cwiseProduct(e_.data) + Kd_.cwiseProduct(e_dot_.data) + C_mat.data + G_mat.data;

    for(size_t i=0; i<n_joints_; i++)
    {
      joints_[i].setCommand(tau_cmd_(i));
    }

    StabilityFactor();
    publish_data();
    print_state();

    M_mat_p = M_mat;
    time_ = time_ + dt_;
  }

  void stopping(const ros::Time& time) override
  {

  }

  void task_via()
  {
    time_ = 0.0;

    for (size_t i = 0; i < n_joints_; i++)
    {
      q_init_(i) = joints_[i].getPosition();
      dq_(i) = dq_end_(i);
      dq_dot_(i) = 0.0;
      dq_ddot_(i) = 0.0;
    }
  }

  void task_init()
  {
    for (size_t i=0; i<n_joints_; i++)
    {
      dq_(i) = 0.0;
      dq_dot_(i) = 0.0;
      dq_ddot_(i) = 0.0;
    }

    dq_end_ = dq_;
  }

  void task_ready()
  {
    for (size_t i=0; i<n_joints_; i++)
    {
      if (i == 2 || i == 4)
      {
        //dq_(i) = trajectory_generator_pos(q_init_(i), PI/2.0, 10.0);
        //dq_dot_(i) = trajectory_generator_vel(q_init_(i), PI/2.0, 10.0);
        //dq_ddot_(i) = trajectory_generator_acc(q_init_(i), PI/2.0, 10.0);

        dq_(i) = PI/2.0;
        dq_dot_(i) = 0.0;
        dq_ddot_(i) = 0.0;
      }
      else
      {
        dq_(i) = q_init_(i);
        dq_dot_(i) = 0.0;
        dq_ddot_(i) = 0.0;
      }
    }

    dq_end_.data = dq_.data;
  }

  void task_freespace()
  {
    KDL::Frame start;
    KDL::Twist target_vel;
    KDL::JntArray cart_cmd;

    cart_cmd.data = Eigen::VectorXd::Zero(3);

    fk_solver_->JntToCart(q_init_, start);

    for (size_t i=0; i<6; i++)
    {
      if(i == 2)
      {
        target_vel(i) = trajectory_generator_vel(start.p(i), 0.122, 10.0);
      }
      else
      {
        target_vel(i) = 0.0;
      }
    }

    ik_vel_solver_->CartToJnt(dq_, target_vel, dq_dot_);

    for (size_t i=0; i<n_joints_; i++)
    {
      dq_(i) = dq_(i) + dq_dot_(i)*dt_;
      dq_ddot_(i) = (dq_dot_(i) - dq_dot_old_(i))/dt_;
      dq_dot_old_(i) = dq_dot_(i);
    }

    dq_end_.data = dq_.data;
  }

  void task_contactspace()
  {
    KDL::Frame start;

    fk_solver_->JntToCart(q_init_, start);

    for (size_t i = 0; i < 6; i++)
    {
      if (i == 0)
      {
        Xc_dot_(i) = trajectory_generator_vel(start.p(i), 0.5, 20.0);
      }
      else if (i == 2)
      {
        Xc_dot_(i) = trajectory_generator_vel(start.p(i), 0.005, 20.0);
      }
      else
      {
        Xc_dot_(i) = 0.0;
      }
    }

    ik_vel_solver_->CartToJnt(dq_, Xc_dot_, dq_dot_);

    for (size_t i=0; i<n_joints_; i++)
    {
      dq_(i) = dq_(i) + dq_dot_(i)*dt_;
      dq_ddot_(i) = (dq_dot_(i) - dq_dot_old_(i))/dt_;
      dq_dot_old_(i) = dq_dot_(i);
    }

    dq_end_ = dq_;
  }

  void task_pos1()
  {
    for (size_t i=0; i<n_joints_; i++)
    {
      dq_(i) = M_PI_4;
      dq_dot_(i) = 0.0;
      dq_ddot_(i) = 0.0;
    }

    dq_end_ = dq_;
  }

  void task_pos2()
  {
    for (size_t i=0; i<n_joints_; i++)
    {
      dq_(i) = -M_PI_4;
      dq_dot_(i) = 0.0;
      dq_ddot_(i) = 0.0;
    }

    dq_end_ = dq_;
  }

  void task_homming()
  {
    for (size_t i = 0; i < n_joints_; i++)
    {
      //dq_(i) = trajectory_generator_pos(q_init_(i), 0.0, 10.0);
      //dq_dot_(i) = trajectory_generator_vel(q_init_(i), 0.0, 10.0);
      //dq_ddot_(i) = trajectory_generator_acc(q_init_(i), 0.0, 10.0);

      dq_(i) = 0.0;
      dq_dot_(i) = 0.0;
      dq_ddot_(i) = 0.0;
    }

    dq_end_.data = dq_.data;
  }

  double trajectory_generator_pos(double dStart, double dEnd, double dDuration)
  {
    double dA0 = dStart;
    double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
    double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
    double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

    return dA0 + dA3*time_*time_*time_ + dA4*time_*time_*time_*time_ + dA5*time_*time_*time_*time_*time_;
  }

  double trajectory_generator_vel(double dStart, double dEnd, double dDuration)
  {
    double dA0 = dStart;
    double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
    double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
    double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

    return 3.0*dA3*time_*time_ + 4.0*dA4*time_*time_*time_ + 5.0*dA5*time_*time_*time_*time_;
  }

  double trajectory_generator_acc(double dStart, double dEnd, double dDuration)
  {
    double dA0 = dStart;
    double dA3 = (20.0*dEnd - 20.0*dStart) / (2.0*dDuration*dDuration*dDuration);
    double dA4 = (30.0*dStart - 30*dEnd) / (2.0*dDuration*dDuration*dDuration*dDuration);
    double dA5 = (12.0*dEnd - 12.0*dStart) / (2.0*dDuration*dDuration*dDuration*dDuration*dDuration);

    return 6.0*dA3*time_ + 12.0*dA4*time_*time_ + 20.0*dA5*time_*time_*time_;
  }

  void GainUpdate()
  {
    if(e_dot_.data.norm() >= epsilon)
    {
      Kp_dot_ = (2.0/(alpha*e_dot_.data.norm()*e_dot_.data.norm())*e_dot_.data*d_.data.transpose() + (1.0/alpha)*M_mat_dot.data).diagonal();
    } else{
      Kp_dot_.setZero();
    }
    Kp_ += Kp_dot_*dt_;
  }

  void StabilityFactor()
  {
    lyapunov_ = 0.5 * e_dot_.data.transpose() * M_mat.data * e_dot_.data;
    lyapunov_ += (alpha / 2.0) * e_.data.transpose() * Kp_.asDiagonal() * e_.data;



    lyapunov_dot[0] = 0.5*e_dot_.data.transpose()*M_mat_dot.data*e_dot_.data;
    lyapunov_dot[0] += e_dot_.data.transpose()*M_mat.data*e_dot_.data;
    lyapunov_dot[0] += (alpha/2.0)*e_.data.transpose()*Kp_dot_.asDiagonal()*e_.data;
    lyapunov_dot[0] += alpha*e_.data.transpose()*Kp_.asDiagonal()*e_dot_.data;

    Eigen::MatrixXd tmp;
    tmp = ((1-alpha)/2.0)*Kp_.asDiagonal();
    tmp -= (alpha/2.0)*Kp_dot_.asDiagonal();

    lyapunov_dot[1] = -e_.data.transpose()*tmp*e_.data;

    tmp = ((1-alpha)/2.0)*Kp_.asDiagonal();
    tmp += Kd_.asDiagonal();

    SufficientCondition_ = tmp.norm();

    tmp -= 0.5*M_mat_dot.data;

    SufficientCondition_ -= (0.5*M_mat_dot.data).norm();

    //lyapunov_dot[1] -= 0.5*e_dot_.data.transpose()*tmp*e_dot_.data;
    //lyapunov_dot[1] += 0.5*d_.data.transpose()*tmp.inverse()*d_.data;
  }

  void publish_data()
  {
    static int loop_count_=0;
    if (loop_count_ >= 9)
    {
      if (controller_state_pub_->trylock())
      {
        controller_state_pub_->msg_.header.stamp = t;
        for(int i=0; i<(n_joints_-1); i++)
        {
          controller_state_pub_->msg_.data[6*i] = e_.data(i);
          controller_state_pub_->msg_.data[6*i+1] = e_dot_.data(i);
          controller_state_pub_->msg_.data[6*i+2] = tau_cmd_.data(i);
          controller_state_pub_->msg_.data[6*i+3] = Kp_(i);
          controller_state_pub_->msg_.data[6*i+4] = Kp_dot_(i);
          controller_state_pub_->msg_.data[6*i+5] = d_.data(i);
        }
        controller_state_pub_->msg_.data[36] = total_time_;
        controller_state_pub_->msg_.data[37] = lyapunov_;
        controller_state_pub_->msg_.data[38] = lyapunov_dot[0];
        controller_state_pub_->msg_.data[39] = SufficientCondition_;
        controller_state_pub_->unlockAndPublish();
      }
      loop_count_=0;
    }
    loop_count_++;
  }

  void print_state()
  {
    static int count = 0;
    if (count >= 99)
    {
      printf("*********************************************************\n\n");
      printf("*** Simulation Time (unit: sec)  ***\n");
      printf("t = %f,\tdt = %f\n", (double)total_time_, dt_);
      printf("\n");

      printf("*** States in Joint Space (unit: deg) ***\n");

      for(int i=0; i < n_joints_; i++)
      {
        printf("Joint ID:%d  ", i+1);
        printf("Kp;%0.3lf, Kd:%0.3lf,  ", Kp_(i), Kd_(i));
        printf("q: %0.3lf, ", q_.data(i) * R2D);
        printf("dq: %0.3lf, ", dq_.data(i) * R2D);
        printf("qdot: %0.3lf, ", q_dot_.data(i) * R2D);
        printf("dqdot: %0.3lf, ", dq_dot_.data(i) * R2D);
        printf("tau: %0.3f,  ", tau_cmd_.data(i));
        printf("\n");
      }
      printf("sensor_ft >> x:%0.3f, y:%0.3f, z:%0.3f, u:%0.3f, v:%0.3f, w:%0.3f\n\n",f_cur_.force.x(), f_cur_.force.y(), f_cur_.force.z(), f_cur_.torque.x(), f_cur_.torque.y(), f_cur_.torque.z() );
      printf("Check Stability Factors:\n");
      printf("Lyapunov:%0.3lf, Lyapunov_dot:%0.3lf, %0.3lf  ", lyapunov_, lyapunov_dot[0], lyapunov_dot[1]);
      printf("SufficientCond:%0.3lf\n", SufficientCondition_);
      printf("Alpha:%0.2lf, Epsilon:%0.2lf\n\n", alpha, epsilon);
      count = 0;
    }
    count++;
  }

private:

  ros::Time t;
  // joint handles
  unsigned int n_joints_;
  std::vector<std::string> joint_names_;
  std::vector<hardware_interface::JointHandle> joints_;

  realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;
  ros::Subscriber sub_command_;
  ros::Subscriber sub_forcetorque_sensor_;

  boost::scoped_ptr<realtime_tools::RealtimePublisher<singlearm_controller::ControllerMatlabState>> controller_state_pub_;

  std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

  // kdl
  KDL::Tree 	kdl_tree_;
  KDL::Chain	kdl_chain_;
  boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
  boost::scoped_ptr<KDL::ChainFkSolverPos> fk_solver_;
  boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

  KDL::JntSpaceInertiaMatrix M_mat, M_mat_p, M_mat_dot;
  KDL::JntArray C_mat;
  KDL::JntArray G_mat;
  KDL::Vector gravity_;

  KDL::Jacobian J_;

  // tdc gain
  Eigen::VectorXd Kp_, Kd_;
  Eigen::VectorXd Kp_dot_;

  // cmd, state
  KDL::JntArray q_init_;
  KDL::JntArray tau_cmd_;
  KDL::JntArray dq_, dq_dot_, dq_dot_old_, dq_ddot_;
  KDL::JntArray dq_end_;
  KDL::JntArray q_, q_dot_;
  KDL::JntArray e_, e_dot_;
  KDL::JntArray d_, d_Cart_;
  KDL::Wrench f_cur_;
  KDL::Twist Xc_dot_;

  double dt_;
  double time_;
  double total_time_;

  double lyapunov_;
  double lyapunov_dot[2];
  double SufficientCondition_;
  double alpha, epsilon;

  int experiment_mode_;
};
}

PLUGINLIB_EXPORT_CLASS(singlearm_controller::VariableImpedanceController, controller_interface::ControllerBase)

