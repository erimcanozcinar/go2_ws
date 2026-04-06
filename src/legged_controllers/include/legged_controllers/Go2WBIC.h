#ifndef GO2_WBIC_H
#define GO2_WBIC_H

// ROS Control includes
#include <legged_common/hardware_interface/HybridJointInterface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>
#include <controller_interface/multi_interface_controller.h>
// #include <controller_interface/controller.h>
// #include <hardware_interface/robot_hw.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/imu_sensor_interface.h>

// Other includes
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <vector>

// Publishing and msgs
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64MultiArray.h>

#include <eigen3/Eigen/Dense>

#include "legged_trajectory/Trajectory.hpp"
#include "legged_estimator/StateEstimator.hpp"
#include "legged_controllers/BalanceController/BalanceController.hpp"
#include "legged_controllers/MPC/MPC.hpp"
#include "legged_controllers/WBC/WBIC.hpp"
#include "rbdyn/RigidBodyModel.hpp"

#include <cassert>
#include <ctime>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>

#include <chrono>

namespace legged
{
class Go2WBIC : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface, ContactSensorInterface>
{

typedef Eigen::Matrix<double, 12, 1> Vec12;

public:
    Go2WBIC();
    ~Go2WBIC();

    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n);
    void update(const ros::Time& time, const ros::Duration& dt);
    void starting(const ros::Time& time);

    std::vector< std::string > joint_names_;

    std::vector<HybridJointHandle> joints_;
    std::vector<ContactSensorHandle> loadcells_;
    hardware_interface::ImuSensorHandle imu_;

    unsigned int n_joints_;

    ros::ServiceClient client;

private:
    ros::Time initial_time;
    ros::Time params_last_published;
    double real_time;

    legged::HybridJointInterface* hw_eff;

    /*#region: Declerations*/
    RigidBodyModel go2Model;
    RobotStates lowStates;
    EstimatorData estResult;
    
    Robot* unitreeGo2 = nullptr;
    Trajectory* traj = nullptr;    
    Estimator* estimator = nullptr;
    BalanceController* balanceController = nullptr;
    ModelPredictiveControl* mpc = nullptr;
    WBIC* wbic = nullptr;
    
    Eigen::Matrix3d kpJoint[4], kdJoint[4];
    Eigen::VectorXd Kp, Kd;

    Eigen::Vector3d qRef[4], dqRef[4];
    Eigen::Vector3d Tau_ff[4];
    Eigen::VectorXd Tau_go2;
    Eigen::VectorXd qJoint_ref, dqJoint_ref;

    bool dampingMode = false;
    bool first = false;

    /*#endregion*/

    void commandCB(const std_msgs::Float64MultiArray& msg);
}; // class

} // namespace

#endif // LEGGED_GO2_TABLET_CONTROLLER_H