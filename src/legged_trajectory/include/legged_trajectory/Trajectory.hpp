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
    EstimatorData* _est;
    double dT;   

    ComTrajectory comTraj;
    std::array<CycloidSwingTrajectory, 4> footSwingTraj;

    
    std::array<Eigen::Vector3d, 4> p0, pf;

    double cmdJoy[4] = {0.0, 0.0, 0.0, initZc};

    Eigen::Vector3d Vcmd;
    
    double yShift[4] = {-0.08, 0.08, 0.08, -0.08};
        
    public:
    GamePad* jStick;
    Gait* gait;

    Eigen::Vector3d Pcom, Vcom, Acom;
    Eigen::Vector3d Pcom_des, Vcom_des, Acom_des;

    std::array<Eigen::Vector3d, 4> pFoot, pFootWorld, pHip, pFoot_initial, pRobotFrame, pYawCorrected;
    std::array<Eigen::Vector3d, 4> vFoot, vFootWorld;
    std::array<Eigen::Vector3d, 4> aFoot, aFootWorld;

    std::array<int, 4> conState;

    Trajectory(EstimatorData* _estData, double _dT = 0.001);
    void trajGeneration();
    DesiredStates* getDesiredStates() {return &_desiredStates;}
};

/* Declerations */



#endif