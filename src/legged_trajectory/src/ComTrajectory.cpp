#include "legged_trajectory/ComTrajectory.hpp"


ComTrajectory::ComTrajectory(double _dt) : dT(_dt) {
    std::cout << "Sampling rate of CoM trajectory: " << 1/dT << " Hz" << std::endl;
    comPos = Eigen::Vector3d::Zero();
    comVel = Eigen::Vector3d::Zero();
    comAcc = Eigen::Vector3d::Zero();
    comPosPrev = Eigen::Vector3d::Zero();
    cmdVelPrev = Eigen::Vector3d::Zero();
}

void ComTrajectory::comTrajPlanner(Eigen::Vector2d cmdVel, Eigen::Vector2d Vel, double cmdHeight, double stancePeriod) {
    comVel.topRows(2) = cmdVel;
    comPos(0) = numIntegral(cmdVel(0), cmdVelPrev(0), comPosPrev(0), dT);
    comPos(1) = numIntegral(cmdVel(1), cmdVelPrev(1), comPosPrev(1), dT);
    comPos(2) = cmdHeight;
    comAcc(0) = Numdiff(cmdVel(0), cmdVelPrev(0), dT);
    comAcc(1) = Numdiff(cmdVel(1), cmdVelPrev(1), dT);
    cmdVelPrev.topRows(2) = cmdVel;
    comPosPrev = comPos;
}

void ComTrajectory::calcStride(Eigen::Vector3d cmdVel, Eigen::Vector2d Vel, double swingTimeRemaining, double stancePeriod) {
    Str(0) = Vel(0)*swingTimeRemaining + Vel(0)*stancePeriod*0.5 + 0.03*(Vel(0)-cmdVel(0)) + (0.5*zCom/9.81)*(Vel(1)*cmdVel(2));
    Str(1) = Vel(1)*swingTimeRemaining + Vel(1)*stancePeriod*0.5 + 0.03*(Vel(1)-cmdVel(1)) + (0.5*zCom/9.81)*(-Vel(0)*cmdVel(2));
    Str(2) = 0.0;

    // Str(0) = fminf(fmaxf(Str(0), -0.5), 0.5);
    // Str(1) = fminf(fmaxf(Str(1), -0.5), 0.5);
}