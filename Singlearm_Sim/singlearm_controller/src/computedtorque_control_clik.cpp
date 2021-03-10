//
// Created by june on 19. 12. 5..
//


// from ros-control meta packages
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
#include <Eigen/Dense>
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


#include <cmath>
#define _USE_MATH_DEFINES

#include "SerialManipulator.h"
#include "Controller.h"


#define D2R M_PI/180.0
#define R2D 180.0/M_PI
#define num_taskspace 6
#define SaveDataMax 97

#define A 0.10
#define b1 0.551
#define b2 -0.516
#define b3 0.643
#define f 0.5

namespace  singlearm_controller
{
    class ComputedTorque_Control_CLIK : public controller_interface::Controller<hardware_interface::EffortJointInterface>
    {
    public:
      ComputedTorque_Control_CLIK()
      {

      }
      ~ComputedTorque_Control_CLIK()
      {
        sub_command_.shutdown();
      }
      bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
      {
          // ********* 1. Get joint name / gain from the parameter server *********
          // 1.0 Control objective & Inverse Kinematics mode
          if (!n.getParam("ctr_obj", ctr_obj_))
          {
              ROS_ERROR("Could not find control objective");
              return false;
          }

          if (!n.getParam("ik_mode", ik_mode_))
          {
              ROS_ERROR("Could not find control objective");
              return false;
          }

          // 1.1 Joint Name
          if (!n.getParam("joints", joint_names_))
          {
              ROS_ERROR("Could not find joint name");
              return false;
          }
          n_joints_ = joint_names_.size();

          if (n_joints_ == 0)
          {
              ROS_ERROR("List of joint names is empty.");
              return false;
          }
          else
          {
              ROS_INFO("Found %d joint names", n_joints_);
              for (int i = 0; i < n_joints_; i++)
              {
                  ROS_INFO("%s", joint_names_[i].c_str());
              }
          }

          // 1.2 Gain
          // 1.2.1 Joint Controller
          Kp_.resize(n_joints_);
          Kd_.resize(n_joints_);
          Ki_.resize(n_joints_);

          std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_);

          for (size_t i = 0; i < n_joints_; i++)
          {
              std::string si = std::to_string(i + 1);
              if (n.getParam("/singlearm/computedtorque_control_clik/gains/joint" + si + "/pid/p", Kp[i]))
              {
                  Kp_(i) = Kp[i];
              }
              else
              {
                  std::cout << "/singlearm/computedtorque_control_clik/gains/joint" + si + "/pid/p" << std::endl;
                  ROS_ERROR("Cannot find pid/p gain");
                  return false;
              }

              if (n.getParam("/singlearm/computedtorque_control_clik/gains/joint" + si + "/pid/i", Ki[i]))
              {
                  Ki_(i) = Ki[i];
              }
              else
              {
                  ROS_ERROR("Cannot find pid/i gain");
                  return false;
              }

              if (n.getParam("/singlearm/computedtorque_control_clik/gains/joint" + si + "/pid/d", Kd[i]))
              {
                  Kd_(i) = Kd[i];
              }
              else
              {
                  ROS_ERROR("Cannot find pid/d gain");
                  return false;
              }
          }

          // 1.2.2 Closed-loop Inverse Kinematics Controller
          if (ctr_obj_ == 1)
          {
              if (!n.getParam("/singlearm/computedtorque_control_clik/clik_gain/K_regulation", K_regulation_))
              {
                  ROS_ERROR("Cannot find clik regulation gain");
                  return false;
              }
          }

          else if (ctr_obj_ == 2)
          {
              if (!n.getParam("/singlearm/computedtorque_control_clik/clik_gain/K_tracking", K_tracking_))
              {
                  ROS_ERROR("Cannot find clik tracking gain");
                  return false;
              }
          }

          // 2. ********* urdf *********
          urdf::Model urdf;
          if (!urdf.initParam("robot_description"))
          {
              ROS_ERROR("Failed to parse urdf file");
              return false;
          }
          else
          {
              ROS_INFO("Found robot_description");
          }

          // 3. ********* Get the joint object to use in the realtime loop [Joint Handle, URDF] *********
          for (int i = 0; i < n_joints_; i++)
          {
              try
              {
                  joints_.push_back(hw->getHandle(joint_names_[i]));
              }
              catch (const hardware_interface::HardwareInterfaceException &e)
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

          // 4. ********* KDL *********
          // 4.1 kdl parser
          if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
          {
              ROS_ERROR("Failed to construct kdl tree");
              return false;
          }
          else
          {
              ROS_INFO("Constructed kdl tree");
          }

          // 4.2 kdl chain
          std::string root_name, tip_name1;
          if (!n.getParam("root_link", root_name))
          {
              ROS_ERROR("Could not find root link name");
              return false;
          }
          if (!n.getParam("tip_link1", tip_name1))
          {
              ROS_ERROR("Could not find tip link name");
              return false;
          }

          if (!kdl_tree_.getChain(root_name, tip_name1, kdl_chain_))
          {
              ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
              ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name1);
              ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
              ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
              ROS_ERROR_STREAM("  The segments are:");

              KDL::SegmentMap segment_map = kdl_tree_.getSegments();
              KDL::SegmentMap::iterator it;

              for (it = segment_map.begin(); it != segment_map.end(); it++)
                  ROS_ERROR_STREAM("    " << (*it).first);

              return false;
          }

          else
          {
              ROS_INFO("Got kdl chain");
          }

          gravity_ = KDL::Vector::Zero();
          gravity_(2) = -9.81; // 0: x-axis 1: y-axis 2: z-axis

          id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));

          // 4.4 jacobian solver 초기화
          jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

          // 4.5 forward kinematics solver 초기화
          fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

          // ********* 5. 각종 변수 초기화 *********

          // 5.1 KDL Vector 초기화 (사이즈 정의 및 값 0)
          tau_d_.data = Eigen::VectorXd::Zero(n_joints_);

          dq_.data = Eigen::VectorXd::Zero(n_joints_);
          dq_dot_.data = Eigen::VectorXd::Zero(n_joints_);
          dq_ddot_.data = Eigen::VectorXd::Zero(n_joints_);

          q_.data = Eigen::VectorXd::Zero(n_joints_);
          q_dot_.data = Eigen::VectorXd::Zero(n_joints_);

          e_.data = Eigen::VectorXd::Zero(n_joints_);
          e_dot_.data = Eigen::VectorXd::Zero(n_joints_);

          // 5.2 KDL Matrix 초기화 (사이즈 정의 및 값 0)
          J_.resize(kdl_chain_.getNrOfJoints());
          // J_inv_.resize(kdl_chain_.getNrOfJoints());
          M_.resize(kdl_chain_.getNrOfJoints());
          C_.resize(kdl_chain_.getNrOfJoints());
          G_.resize(kdl_chain_.getNrOfJoints());

          // ********* 6. ROS 명령어 *********
          // 6.1 publisher
          controller_state_pub_.reset(new realtime_tools::RealtimePublisher<singlearm_controller::ControllerJointState>(n, "state", 1));
          controller_state_pub_->msg_.header.stamp = ros::Time::now();
          for (size_t i=0; i<n_joints_; i++)
          {
            controller_state_pub_->msg_.name.push_back(joint_names_[i]);
            controller_state_pub_->msg_.q.push_back(0.0);
            controller_state_pub_->msg_.dq.push_back(0.0);
            controller_state_pub_->msg_.qdot.push_back(0.0);
            controller_state_pub_->msg_.dqdot.push_back(0.0);
            controller_state_pub_->msg_.ctrl_input.push_back(0.0);
          }

          // 6.2 subsriber
          commands_buffer_.writeFromNonRT(std::vector<double>(n_joints_, 0.0));
          sub_command_ = n.subscribe<std_msgs::Float64MultiArray>("command",1, &ComputedTorque_Control_CLIK::commandCB,this);
          event = 0;

          return true;
      }

      void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
      {
          if (msg->data.size() != n_joints_)
          {
              ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match DOF of Task Space (" << 2 << ")! Not executing!");
              return;
          } else{
            commands_buffer_.writeFromNonRT(msg->data);
            event = 1; // subscribe 받기 전: 0
            // subscribe 받은 후: 1
          }
      }

      void starting(const ros::Time &time) override
      {
        t = time;

        cManipulator = new SerialManipulator;
        Control = new HYUControl::Controller(cManipulator);

        cManipulator->UpdateManipulatorParam();

        Control->SetPIDGain(Kp_.data, Kd_.data, Ki_.data);

        dq_.data.setZero();
        dq_dot_.data.setZero();
        dq_ddot_.data.setZero();

        ROS_INFO("Starting Computed Torque Controller with Closed-Loop Inverse Kinematics");
      }

      void update(const ros::Time &time, const ros::Duration &period) override
      {
          // ********* 0. Get states from gazebo *********
          // 0.1 sampling time
          t = time;

          std::vector<double> &commands = *commands_buffer_.readFromRT();

          // 0.2 joint state
          for (int i = 0; i < n_joints_; i++)
          {
              q_.data(i) = joints_[i].getPosition();
              q_dot_.data(i) = joints_[i].getVelocity();
              dq_.data(i) = commands[i]*D2R;
          }

          e_.data = dq_.data - q_.data;
          e_dot_.data = dq_dot_.data - q_dot_.data;

          // *** 3.2 Compute model(M,C,G) ***
          id_solver_->JntToMass(q_, M_);
          id_solver_->JntToCoriolis(q_, q_dot_, C_);
          id_solver_->JntToGravity(q_, G_);

          jnt_to_jac_solver_->JntToJac(q_, J_);

          double q_arr[n_joints_];
          double qdot_arr[n_joints_];

          Eigen::Map<Eigen::VectorXd>(q_arr, n_joints_) = q_.data;
          Eigen::Map<Eigen::VectorXd>(qdot_arr, n_joints_) = q_dot_.data;

          cManipulator->pKin->PrepareJacobian(q_arr);
          cManipulator->pDyn->PrepareDynamics(q_arr, qdot_arr);

          cManipulator->pDyn->MG_Mat_Joint(M_poe.data, G_poe.data);


          // *** 3.3 Apply Torque Command to Actuator ***
          aux_d_.data = M_.data * dq_ddot_.data + Kp_.data.cwiseProduct(e_.data) + Kd_.data.cwiseProduct(e_dot_.data);
          comp_d_.data = C_.data + G_.data;
          tau_d_.data = aux_d_.data + comp_d_.data;


          for (int i = 0; i < n_joints_; i++)
          {
              joints_[i].setCommand(tau_d_(i));
          }

          publish_data();

          print_state();
      }

        void stopping(const ros::Time &time) override
        {
            delete Control;
            delete cManipulator;
        }


      void publish_data()
      {
        static int loop_count_=0;
        if (loop_count_ >= 9)
        {
          if (controller_state_pub_->trylock())
          {
            controller_state_pub_->msg_.header.stamp = t;
            for(int i=0; i<n_joints_; i++)
            {
              controller_state_pub_->msg_.q[i] = q_.data(i);
              controller_state_pub_->msg_.dq[i] = dq_.data(i);
              controller_state_pub_->msg_.qdot[i] = q_dot_.data(i);
              controller_state_pub_->msg_.dqdot[i] = dq_dot_.data(i);
              controller_state_pub_->msg_.ctrl_input[i] = tau_d_.data(i);
            }
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
          printf("t = %f\n", (double)t.toSec());
          printf("\n");

          printf("*** Command from Subscriber in Joint-Space (unit: deg) ***\n");
          if (event == 0)
          {
              printf("No Active!!!\n");
          }
          else
          {
              printf("Active!!!\n");
          }

          printf("*** States in Joint Space (unit: deg) ***\n");

          //Control->GetPIDGain(Kp_.data, Kd_.data, Ki_.data);
          for(int i=0; i < n_joints_; i++)
          {
              printf("Joint ID:%d \t", i+1);
              printf("Kp;%0.3lf, Kd:%0.3lf,  ", Kp_.data(i), Kd_.data(i));
              printf("q: %0.3lf, ", q_.data(i) * R2D);
              printf("dq: %0.3lf, ", dq_.data(i) * R2D);
              printf("qdot: %0.3lf, ", q_dot_.data(i) * R2D);
              printf("dqdot: %0.3lf, ", dq_dot_.data(i) * R2D);
              printf("tau: %0.3f,  ", tau_d_.data(i));
              printf("\n");
          }
          count = 0;
        }
        count++;
      }

    private:
      // others
      int ctr_obj_;
      int ik_mode_;
      int event;

      ros::Time t;
      //Joint handles
      unsigned int n_joints_;
      std::vector<std::string> joint_names_;
      std::vector<hardware_interface::JointHandle> joints_;

      realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;
      ros::Subscriber sub_command_;

      boost::scoped_ptr<realtime_tools::RealtimePublisher<singlearm_controller::ControllerJointState>> controller_state_pub_;

      std::vector<urdf::JointConstSharedPtr> joint_urdfs_;

      // kdl
      KDL::Tree kdl_tree_;
      KDL::Chain kdl_chain_;
      boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;    //Solver to compute the forward kinematics (position)
      boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;       //Solver to compute the jacobian
      boost::scoped_ptr<KDL::ChainDynParam> id_solver_;                     //Solver To compute the inverse dynamics

      // kdl M,C,G
      KDL::JntSpaceInertiaMatrix M_, M_poe; // intertia matrix
      KDL::JntArray C_, C_poe;              // coriolis
      KDL::JntArray G_, G_poe;              // gravity torque vector
      KDL::Vector gravity_;

      // kdl and Eigen Jacobian
      KDL::Jacobian J_;
      KDL::Jacobian J_poe;

      // Joint Space State
      KDL::JntArray dq_, dq_dot_, dq_ddot_;
      KDL::JntArray q_, q_dot_;
      KDL::JntArray e_, e_dot_;

      // Torque
      KDL::JntArray aux_d_;
      KDL::JntArray comp_d_;
      KDL::JntArray tau_d_;

      // gains
      KDL::JntArray Kp_, Ki_, Kd_;
      double K_regulation_, K_tracking_;


      SerialManipulator *cManipulator;
      HYUControl::Controller *Control;
    };
}

PLUGINLIB_EXPORT_CLASS(singlearm_controller::ComputedTorque_Control_CLIK,controller_interface::ControllerBase)