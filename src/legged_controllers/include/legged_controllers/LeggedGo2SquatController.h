#ifndef LEGGED_SQUAT_CONTROLLER_H
#define LEGGED_SQUAT_CONTROLLER_H

// ROS Control includes
#include <controller_interface/multi_interface_controller.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

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

#include <eigen3/Eigen/Dense>

#include <cassert>
#include <ctime>
#include <memory>

#include <chrono>

namespace legged
{
class LeggedGo2SquatController : public controller_interface::MultiInterfaceController<HybridJointInterface>
{

typedef Eigen::Matrix<double, 12, 1> Vec12;

public:
    LeggedGo2SquatController();
    ~LeggedGo2SquatController();

    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n);
    void update(const ros::Time& time, const ros::Duration& dt);
    void starting(const ros::Time& time);

    std::vector< std::string > joint_names_;
    // std::vector< hardware_interface::JointHandle > joints_;

    std::vector<HybridJointHandle> joints_;

    realtime_tools::RealtimeBuffer<std::vector<double>> commands_buffer_;
    unsigned int n_joints_;

    ros::ServiceClient client;

private:
    enum RobotTorsoState{
        LOWERING,
        STATIONARY,
        RAISING,
    };

    bool got_body_height = false;
    bool fixed_Cx = false;

    ros::Time initial_time;
    double real_time;

    RobotTorsoState robot_state = RobotTorsoState::STATIONARY;
    bool can_state_change = true;
    double change_start_time = 0.0;
    double change_duration = 5.0;

    legged::HybridJointInterface* hw_eff;

    ros::Subscriber sub_command_;

    double Kp[12];
    double Kd[12];
    double Ki[12];

    Vec12 sitting_configuration;
    Vec12 standing_configuration;
    Vec12 starting_configuration;
    Vec12 ql_ref, dql_ref, prev_ql_ref;

    Eigen::Vector3d Pfoot_LF, Pfoot_RF, Pfoot_LB, Pfoot_RB;
    Eigen::Vector3d Q_LF, Q_RF, Q_LB, Q_RB;
    Eigen::Vector3d Pcom, torsoOrient;

    double Cx, Cz, Cz_init, Cx_init;

    void commandCB(const std_msgs::Float64MultiArray& msg);
}; // class

} // namespace

#endif // LEGGED_SQUAT_CONTROLLER_H