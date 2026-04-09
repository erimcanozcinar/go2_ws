#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include "eigen3/Eigen/Dense"
#include "legged_common/GamePad.hpp"
#include "legged_trajectory/GaitScheduler.hpp"
#include "legged_trajectory/ComTrajectory.hpp"
#include "legged_trajectory/FootSwingTrajectory.hpp"
#include "legged_common/kinematics.hpp"
#include "legged_common/DataTypes.hpp"

class Trajectory : public Robot {
    private:
    DesiredStates _desiredStates;
    double dT;   

    ComTrajectory comTraj;
    std::array<CycloidSwingTrajectory, 4> footSwingTraj;

    
    std::array<Eigen::Vector3d, 4> p0, pf;

    Eigen::Vector3d Vcmd;

    double yaw_turn_rate;
    
    double yShift[4] = {-0.08, 0.08, 0.08, -0.08};
        
    public:
    GamePad jStick;
    Gait* gait;

    double cmdJoyF[22] = {0}, pre_cmdJoyF[22] = {0};

    Eigen::Vector3d Pcom, Vcom, Acom;
    Eigen::Vector3d Pcom_des, Vcom_des, Acom_des;
    double Yaw_des, dYaw_des;
    std::array<Eigen::Vector3d, 4> pFoot, pFootWorld, pHip, pFoot_initial, pRobotFrame, pYawCorrected;
    std::array<Eigen::Vector3d, 4> vFoot, vFootWorld;
    std::array<Eigen::Vector3d, 4> aFoot, aFootWorld;

    std::array<int, 4> conState;

    Trajectory(double _dT = 0.001);
    void trajGeneration(const Eigen::Vector3d& Vel, const std::array<Eigen::Vector3d, 4>& pFoot_fk, const Eigen::Vector3d& Pcom_act, const Eigen::Matrix3d& rbody);
    DesiredStates* getDesiredStates() {return &_desiredStates;}
};

/* Declerations */



#endif