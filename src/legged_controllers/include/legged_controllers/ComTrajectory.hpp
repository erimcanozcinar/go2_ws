#ifndef COMTRAJECTORY_HPP
#define COMTRAJECTORY_HPP

#include "eigen3/Eigen/Dense"
#include "legged_common/matTools.hpp"


class ComTrajectory {
    public:
        double zCom;
        double dT;

        ComTrajectory(double  _dt = 0.001);
        void comTrajPlanner(Eigen::Vector2d cmdVel, Eigen::Vector2d Vel, double cmdHeight, double stancePeriod);
        void calcStride(Eigen::Vector3d cmdVel, Eigen::Vector2d Vel, double swingTimeRemaining, double stancePeriod);
        double getFootHeight() const { return Fh; }
        Eigen::Vector3d getComPos() const { return comPos; }
        Eigen::Vector3d getComVel() const { return comVel; }
        Eigen::Vector3d getComAcc() const { return comAcc; }
        Eigen::Vector3d getStrideLength() const { return Str; }
    private:
        Eigen::Vector3d comPos;
        Eigen::Vector3d comVel;
        Eigen::Vector3d comAcc;
        Eigen::Vector3d comPosPrev;
        Eigen::Vector3d cmdVelPrev;
        Eigen::Vector3d Str;
        double Fh = 0.08; 

};

#endif