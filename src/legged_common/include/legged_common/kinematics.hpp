#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include "eigen3/Eigen/Dense"
#include "legged_common/matTools.hpp"
#include "legged_common/parameters.hpp"

enum class FrameType { HIP, BASE };

class Leg
{
    private:

    double _abadLinkLength, _thighLinkLength, _calfLinkLength;

    public:
    int _xSign, _ySign;
    Eigen::Vector3d pHipAA, pHipFE, pKneeFE, pFootEnd;
    Eigen::Vector3d footPosNormalStand, footPosBodyNormalStand, footFrameBody;
    Eigen::Vector3d pHip2Body;

    Leg(int leg);
    Eigen::Vector3d calcFootPos(const Eigen::Vector3d& Q, FrameType frame);
    Eigen::Vector3d calcFootVel(const Eigen::Vector3d& Q, const Eigen::Vector3d& dQ);
    Eigen::Matrix3d calcLegJac(const Eigen::Vector3d& q);
    Eigen::Vector3d calcJointPos(const Eigen::Vector3d& pfoot);
    Eigen::Vector3d calcJointVel(const Eigen::Vector3d& vfoot, const Eigen::Vector3d& q);

};

class Robot 
{
    private:
    public:
        Leg _LF, _RF, _LB, _RB;

        Robot();
        ~Robot(){}
        Eigen::Matrix3d calcBodyJac(const Eigen::Vector3d& rpy);
};

#endif