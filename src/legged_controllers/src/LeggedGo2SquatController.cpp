#include <legged_controllers/LeggedGo2SquatController.h>
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
  LeggedGo2SquatController::LeggedGo2SquatController() {}
  LeggedGo2SquatController::~LeggedGo2SquatController() {sub_command_.shutdown();}

  bool LeggedGo2SquatController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &n)
  {
    ROS_INFO("SquatController Go2 | Init");

    hw_eff   = robot_hw->get<HybridJointInterface>();
    
    // // LF (gain of the knees were 0.9)
    Kp[0] = 50; Kd[0] = 5; Ki[0] = 0; // Hip AA
    Kp[1] = 50; Kd[1] = 5; Ki[1] = 0; // Hip FE
    Kp[2] = 50; Kd[2] = 5; Ki[2] = 0; // Knee FE 

    // LB (gain of the knees were 0.9)
    Kp[3] = 50; Kd[3] = 5; Ki[3] = 0; // Knee FE
    Kp[4] = 50; Kd[4] = 5; Ki[4] = 0; // Hip FE
    Kp[5] = 50; Kd[5] = 5; Ki[5] = 0; // Hip AA

    // RB (gain of the knees were 0.9)
    Kp[6] = 50; Kd[6] = 5; Ki[6] = 0; // Hip AA
    Kp[7] = 50; Kd[7] = 5; Ki[7] = 0; // Hip FE
    Kp[8] = 50; Kd[8] = 5; Ki[8] = 0; // Knee FE

    // RF (gain of the knees were 0.9)
    Kp[9] = 50; Kd[9] = 5; Ki[9] = 0; // Knee FE
    Kp[10] = 50; Kd[10] = 5; Ki[10] = 0; // Hip FE
    Kp[11] = 50; Kd[11] = 5; Ki[11] = 0; // Hip AA
    
    joint_names_.push_back("LF_HAA");
    joint_names_.push_back("LF_HFE");
    joint_names_.push_back("LF_KFE");

    joint_names_.push_back("LH_KFE");
    joint_names_.push_back("LH_HFE");
    joint_names_.push_back("LH_HAA");

    joint_names_.push_back("RH_HAA");
    joint_names_.push_back("RH_HFE");
    joint_names_.push_back("RH_KFE");

    joint_names_.push_back("RF_KFE");
    joint_names_.push_back("RF_HFE");
    joint_names_.push_back("RF_HAA");

    n_joints_ = joint_names_.size();

    for(unsigned int i=0; i<n_joints_; i++)
    {
      const auto& joint_name = joint_names_[i];

      try
      {
        joints_.push_back(hw_eff->getHandle(joint_name));
      }
      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }

    }

    commands_buffer_.writeFromNonRT(std::vector<double>(2, 0.0));

    sub_command_ = n.subscribe("legged_squat_command", 1, &LeggedGo2SquatController::commandCB, this);

    return true;
  }

  void LeggedGo2SquatController::starting(const ros::Time& time)
  {
    ROS_INFO("SquatController Go2 | Starting");

    // sitting_configuration << 0.0, 1.36, -2.65, -2.65, 1.36, -0.2, 0.2, 1.36, -2.65, -2.65, 1.36, 0.0;
    // standing_configuration << 0.0, 0.67, -1.3, -1.3, 0.67, 0.0, 0.0, 0.67, -1.3, -1.3, 0.67, 0.0;

    sitting_configuration << 0.0, 1.36, -2.65, -2.65, 1.36, 0.2, -0.2, 1.36, -2.65, -2.65, 1.36, 0.0;
    standing_configuration << 0.0, 0.67, -1.3, -1.3, 0.67, 0.0, 0.0, 0.67, -1.3, -1.3, 0.67, 0.0;
    
    robot_state = RobotTorsoState::STATIONARY;
    can_state_change = true;

    for(int i=0; i<12; i++){
      starting_configuration(i) = joints_[i].getPosition();
      prev_ql_ref(i) = joints_[i].getPosition();
    }

    std::vector<double> current_command(2, 0.0);

    commands_buffer_.initRT(current_command);

    initial_time = ros::Time::now();
  }

  void LeggedGo2SquatController::update(const ros::Time& time, const ros::Duration& period)
  {
    std::vector<double> & commands = *commands_buffer_.readFromRT();
    real_time = (time - initial_time).toSec();
    
    if(can_state_change && commands[0]){
      ROS_INFO("RAISING!");
      robot_state = RobotTorsoState::RAISING;
      change_start_time = real_time;
      can_state_change = false;
      Cz_init = Cz;
    } else if (can_state_change && commands[1]) {
      ROS_INFO("LOWERING!");
      robot_state = RobotTorsoState::LOWERING;
      change_start_time = real_time;
      can_state_change = false;
      Cz_init = Cz;
    }

    if(robot_state == RobotTorsoState::RAISING){
      if(real_time - change_start_time < change_duration){
        ql_ref = (standing_configuration - starting_configuration)/(change_duration) * (real_time - change_start_time) + starting_configuration;
      } else {
        can_state_change = true;
        robot_state = RobotTorsoState::STATIONARY;
        starting_configuration = standing_configuration;
      }
    } else if(robot_state == RobotTorsoState::LOWERING){
      if(real_time - change_start_time < change_duration){
        ql_ref = (sitting_configuration - starting_configuration)/(change_duration) * (real_time - change_start_time) + starting_configuration;
      } else {
        can_state_change = true;
        robot_state = RobotTorsoState::STATIONARY;
        starting_configuration = sitting_configuration;
      }
    } else {
      ql_ref = starting_configuration;
    }

    for(unsigned int i=0; i<n_joints_; i++)
    {   
        dql_ref(i) = ((ql_ref(i) - prev_ql_ref(i)) / period.toSec());
        prev_ql_ref(i) = ql_ref(i);

        joints_[i].setCommand(ql_ref(i), dql_ref(i), 50.0, 5.0, 0);


        // if(robot_state == RobotTorsoState::LOWERING){
        //   joints_[i].setCommand(0, 0, 0, 5, 0);
        // }else{
        //   joints_[i].setCommand(ql_ref(i), dql_ref(i), Kp[i], Kd[i], 0);
        // }
    }      
  }

  void LeggedGo2SquatController::commandCB(const std_msgs::Float64MultiArray& msg)
  {
    commands_buffer_.writeFromNonRT(msg.data);
  }

} // legged

PLUGINLIB_EXPORT_CLASS( legged::LeggedGo2SquatController, controller_interface::ControllerBase)