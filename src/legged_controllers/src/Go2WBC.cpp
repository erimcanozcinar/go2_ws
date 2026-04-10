#include <legged_controllers/Go2WBC.h>
#include <pluginlib/class_list_macros.hpp>

namespace legged
{

  /**
   * \brief Forward command controller for a set of effort controlled joints (torque or force).
   *
   * This class forwards the commanded efforts down to a set of joints.
   *
   * \section ROS interface
   *
   * \param type Must be "JointGroupEffortController".
   * \param joints List of names of the joints to control.
   *
   * Subscribes to:
   * - \b command (std_msgs::Float64MultiArray) : The joint efforts to apply
   */
  Go2WBC::Go2WBC() {}
  Go2WBC::~Go2WBC() {
    delete unitreeGo2;
    delete traj;
    delete estimator;
    delete balanceController;
    delete mpc;
    delete wbc;
  }//sub_command_.shutdown(); }

  bool Go2WBC::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &n)
  {
    ROS_INFO("TabletController | Init");

    go2Model = RigidBodyModel("/home/erim/test_ws/src/legged_examples/legged_unitree/legged_unitree_description/urdf/go2/go2.urdf");
    unitreeGo2 = new Robot();
    traj = new Trajectory(&estResult, 0.002);
    estimator = new Estimator(unitreeGo2, 0.002);
    balanceController = new BalanceController(&estResult);
    mpc = new ModelPredictiveControl(&estResult, 0.002, 25);
    wbc = new WBC(unitreeGo2, &estResult, &lowStates);
    
    lowStates.imu.acc.setZero();
    lowStates.imu.gyro.setZero();
    lowStates.imu.orientation.setZero();
    lowStates.imu.rotationMatrix.setIdentity();

    for(int i=0; i<4; i++) {
        lowStates.qJoint[i].setZero();
        lowStates.dqJoint[i].setZero();
        Tau_ff[i].setZero();
        qRef[i].setZero();
        dqRef[i].setZero();
        kpJoint[i].setZero();
        kdJoint[i].setZero();
    }

    kpCartesian << 400, 0, 0,
                    0, 400, 0,
                    0, 0, 400;
    kdCartesian << 10, 0, 0,
                    0, 10, 0,
                    0, 0, 10;

    qJoint_ref.resize(12);
    qJoint_ref.setZero();
    dqJoint_ref.resize(12);
    dqJoint_ref.setZero();
    Tau_go2.resize(12);
    Tau_go2.setZero();
    Kp.resize(12);
    Kd.resize(12);
    Kp.setZero();
    Kd.setZero();

    tauGravity.resize(18);
    tauInvDyn.resize(18);
    tauGravity.setZero();
    tauInvDyn.setZero();

    hw_eff = robot_hw->get<HybridJointInterface>();

    joint_names_.push_back("RF_HAA");
    joint_names_.push_back("RF_HFE");
    joint_names_.push_back("RF_KFE");

    joint_names_.push_back("LF_HAA");
    joint_names_.push_back("LF_HFE");
    joint_names_.push_back("LF_KFE");

    joint_names_.push_back("RH_HAA");
    joint_names_.push_back("RH_HFE");
    joint_names_.push_back("RH_KFE");

    joint_names_.push_back("LH_HAA");
    joint_names_.push_back("LH_HFE");
    joint_names_.push_back("LH_KFE");

    n_joints_ = joint_names_.size();

    for (unsigned int i = 0; i < n_joints_; i++)
    {
      const auto &joint_name = joint_names_[i];

      try
      {
        joints_.push_back(hw_eff->getHandle(joint_name));
      }
      catch (const hardware_interface::HardwareInterfaceException &e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }
    }

    ROS_INFO("Contact Init!");

    auto *contactInterface = robot_hw->get<ContactSensorInterface>();

    loadcells_.push_back(contactInterface->getHandle("LF_FOOT"));
    loadcells_.push_back(contactInterface->getHandle("RF_FOOT"));
    loadcells_.push_back(contactInterface->getHandle("LH_FOOT"));
    loadcells_.push_back(contactInterface->getHandle("RH_FOOT"));

    imu_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("base_imu");

    ROS_INFO("IMU Done!");

    
    return true;
  }

  void Go2WBC::starting(const ros::Time &time)
  {
    ROS_INFO("TabletController | Starting");

    lowStates.imu.orientation = estimator->orientationOffset(Quat(imu_.getOrientation()[3], 
                                                                 imu_.getOrientation()[0], 
                                                                 imu_.getOrientation()[1], 
                                                                 imu_.getOrientation()[2]));
    lowStates.imu.rotationMatrix = quaternionToRotationMatrix(lowStates.imu.orientation);
    lowStates.imu.acc << imu_.getLinearAcceleration()[0], imu_.getLinearAcceleration()[1], imu_.getLinearAcceleration()[2];
    lowStates.imu.gyro << imu_.getAngularVelocity()[0], imu_.getAngularVelocity()[1], imu_.getAngularVelocity()[2];
    lowStates.qJoint[0] << joints_[0].getPosition(), joints_[1].getPosition(), joints_[2].getPosition();   // RF
    lowStates.qJoint[1] << joints_[3].getPosition(), joints_[4].getPosition(), joints_[5].getPosition();   // LF
    lowStates.qJoint[2] << joints_[6].getPosition(), joints_[7].getPosition(), joints_[8].getPosition();   // RB
    lowStates.qJoint[3] << joints_[9].getPosition(), joints_[10].getPosition(), joints_[11].getPosition(); // LB
    lowStates.dqJoint[0] << joints_[0].getVelocity(), joints_[1].getVelocity(), joints_[2].getVelocity();   // RF
    lowStates.dqJoint[1] << joints_[3].getVelocity(), joints_[4].getVelocity(), joints_[5].getVelocity();   // LF
    lowStates.dqJoint[2] << joints_[6].getVelocity(), joints_[7].getVelocity(), joints_[8].getVelocity();   // RB
    lowStates.dqJoint[3] << joints_[9].getVelocity(), joints_[10].getVelocity(), joints_[11].getVelocity(); // LB
    qRef[0] = lowStates.qJoint[0];
    qRef[1] = lowStates.qJoint[1];
    qRef[2] = lowStates.qJoint[2];
    qRef[3] = lowStates.qJoint[3];
    dqRef[0] = lowStates.dqJoint[0];
    dqRef[1] = lowStates.dqJoint[1];
    dqRef[2] = lowStates.dqJoint[2];
    dqRef[3] = lowStates.dqJoint[3];
    
    initial_time = ros::Time::now();
    params_last_published = ros::Time::now();
  }

  void Go2WBC::update(const ros::Time &time, const ros::Duration &period)
  {
    // std::vector<double> &commands = *commands_buffer_.readFromRT();
    real_time = (time - initial_time).toSec();
    
    /* #region: Get sensor data*/
    lowStates.imu.orientation = estimator->orientationOffset(Quat(imu_.getOrientation()[3], 
                                                                 imu_.getOrientation()[0], 
                                                                 imu_.getOrientation()[1], 
                                                                 imu_.getOrientation()[2]));
    lowStates.imu.rotationMatrix = quaternionToRotationMatrix(lowStates.imu.orientation);
    lowStates.imu.acc << imu_.getLinearAcceleration()[0], imu_.getLinearAcceleration()[1], imu_.getLinearAcceleration()[2];
    lowStates.imu.gyro << imu_.getAngularVelocity()[0], imu_.getAngularVelocity()[1], imu_.getAngularVelocity()[2];
    lowStates.qJoint[0] << joints_[0].getPosition(), joints_[1].getPosition(), joints_[2].getPosition();   // RF
    lowStates.qJoint[1] << joints_[3].getPosition(), joints_[4].getPosition(), joints_[5].getPosition();   // LF
    lowStates.qJoint[2] << joints_[6].getPosition(), joints_[7].getPosition(), joints_[8].getPosition();   // RB
    lowStates.qJoint[3] << joints_[9].getPosition(), joints_[10].getPosition(), joints_[11].getPosition(); // LB
    lowStates.dqJoint[0] << joints_[0].getVelocity(), joints_[1].getVelocity(), joints_[2].getVelocity();   // RF
    lowStates.dqJoint[1] << joints_[3].getVelocity(), joints_[4].getVelocity(), joints_[5].getVelocity();   // LF
    lowStates.dqJoint[2] << joints_[6].getVelocity(), joints_[7].getVelocity(), joints_[8].getVelocity();   // RB
    lowStates.dqJoint[3] << joints_[9].getVelocity(), joints_[10].getVelocity(), joints_[11].getVelocity(); // LB
    /* #endregion */

    /* #region: STATE ESTIMATOR */
    estimator->run(lowStates, traj->conState);
    estResult = estimator->getResult();
    /* #endregion */ 

    /* #region: TRAJECTORY GENERATION WITH JOYSTICK */
    traj->trajGeneration();
    // traj->trajGeneration(traj->Vcom_des, traj->pFoot, traj->Pcom_des, estResult.rWorld2Body);
    /* #endregion */


    /* #region: FORCE DISTRIBUTION*/
    // balanceController->setDesiredStates(traj->getDesiredStates());
    // balanceController->run();

    mpc->setDesiredStates(traj->getDesiredStates());
    mpc->run(traj->gait);

    wbc->setDesiredStates(traj->getDesiredStates());
    wbc->prioritizedTaskExecution();
    qRef[0] = wbc->getJointPosCmd().block<3,1>(0,0) + lowStates.qJoint[0];
    qRef[1] = wbc->getJointPosCmd().block<3,1>(3,0) + lowStates.qJoint[1];
    qRef[2] = wbc->getJointPosCmd().block<3,1>(6,0) + lowStates.qJoint[2];
    qRef[3] = wbc->getJointPosCmd().block<3,1>(9,0) + lowStates.qJoint[3];
    dqRef[0] = wbc->getJointVelCmd().block<3,1>(0,0);
    dqRef[1] = wbc->getJointVelCmd().block<3,1>(3,0);
    dqRef[2] = wbc->getJointVelCmd().block<3,1>(6,0);
    dqRef[3] = wbc->getJointVelCmd().block<3,1>(9,0);
    /* #endregion */

    /* #region: GROUND REACTION FORCE AND FOOT IMPEDANCE */
    footForce[0] = mpc->getFootForces()[0] + (1-traj->conState[0])*kpCartesian*(traj->pFoot[0] - estResult.pFoot[0]) + kdCartesian*(traj->vFoot[0]-estResult.vFoot[0]);
    footForce[1] = mpc->getFootForces()[1] + (1-traj->conState[1])*kpCartesian*(traj->pFoot[1] - estResult.pFoot[1]) + kdCartesian*(traj->vFoot[1]-estResult.vFoot[1]);
    footForce[2] = mpc->getFootForces()[2] + (1-traj->conState[2])*kpCartesian*(traj->pFoot[2] - estResult.pFoot[2]) + kdCartesian*(traj->vFoot[2]-estResult.vFoot[2]);
    footForce[3] = mpc->getFootForces()[3] + (1-traj->conState[3])*kpCartesian*(traj->pFoot[3] - estResult.pFoot[3]) + kdCartesian*(traj->vFoot[3]-estResult.vFoot[3]);
    Tau_ff[0] = unitreeGo2->_RF.calcLegJac(lowStates.qJoint[0]).transpose()*footForce[0];
    Tau_ff[1] = unitreeGo2->_LF.calcLegJac(lowStates.qJoint[1]).transpose()*footForce[1];
    Tau_ff[2] = unitreeGo2->_RB.calcLegJac(lowStates.qJoint[2]).transpose()*footForce[2];
    Tau_ff[3] = unitreeGo2->_LB.calcLegJac(lowStates.qJoint[3]).transpose()*footForce[3];
    /* #endregion */

    for (int i = 0; i < 4; i++) {
      if(traj->conState[i] > 0) {
          kpJoint[i] = Eigen::Vector3d(0,0,0).asDiagonal();
          kdJoint[i] = Eigen::Vector3d(2,2,2).asDiagonal();            
      }else {
          kpJoint[i] = Eigen::Vector3d(3,3,3).asDiagonal();
          kdJoint[i] = Eigen::Vector3d(2,2,2).asDiagonal();
      }
      // kpJoint[i] = Eigen::Vector3d(10,10,10).asDiagonal();
      // kdJoint[i] = Eigen::Vector3d(1,1,1).asDiagonal();
      Kp(i*3) = kpJoint[i](0);
      Kp(i*3+1) = kpJoint[i](1);
      Kp(i*3+2) = kpJoint[i](2);
      Kd(i*3) = kdJoint[i](0);
      Kd(i*3+1) = kdJoint[i](1);
      Kd(i*3+2) = kdJoint[i](2);
    }
    
    Tau_go2 << Tau_ff[0], Tau_ff[1], Tau_ff[2], Tau_ff[3];
    qJoint_ref << qRef[0], qRef[1], qRef[2], qRef[3];
    dqJoint_ref << dqRef[0], dqRef[1], dqRef[2], dqRef[3];


    /* #region: DAMPING MODE */
    if(std::fabs(estResult.rpy(0)) > 70*M_PI/180 || std::fabs(estResult.rpy(1)) > 70*M_PI/180){
      dampingMode = true;
      std::cout << "Damping mode enabled" << std::endl;
    }
    /* #endregion */
    
    for (int i = 0; i < 12; i++) {      
      if(dampingMode) {
        joints_[i].setCommand(0, 0, 0, 2, 0);
      } else {
        joints_[i].setCommand(qJoint_ref(i), dqJoint_ref(i), 0, 2, Tau_go2(i));
      }
    }

  }

} // legged

PLUGINLIB_EXPORT_CLASS(legged::Go2WBC, controller_interface::ControllerBase)