#ifndef DATATYPES_HPP
#define DATATYPES_HPP

#include <eigen3/Eigen/Dense>

using Quat = Eigen::Vector4d;
using RotMat = Eigen::Matrix3d;

struct ImuData {
    Eigen::Vector3d acc; // Linear acceleration in body frame
    Eigen::Vector3d gyro; // Angular velocity in body frame
    Quat orientation; // Orientation (quaternion)
    RotMat rotationMatrix; // Rotation matrix from world to body
};

struct RobotStates {
    ImuData imu;
    Eigen::Vector3d qJoint[4];
    Eigen::Vector3d dqJoint[4];
};

struct EstimatorData {
    Quat orientation;       // Orientation (quaternion)
    Eigen::Vector3d rpy; // Roll, pitch, yaw angles
    RotMat rWorld2Body; // Rotation matrix from world to body
    RotMat rBody2World; // Rotation matrix from body to world

    Eigen::Vector3d pos; // Position in world frame

    Eigen::Vector3d vBody; // Velocity in body frame
    Eigen::Vector3d aBody; // Acceleration in body frame
    Eigen::Vector3d omegaBody; // Angular velocity in body frame
    Eigen::Vector3d domegaBody; // Angular acceleration in body frame

    Eigen::Vector3d vWorld; // Velocity in world frame
    Eigen::Vector3d aWorld; // Acceleration in world frame
    Eigen::Vector3d omegaWorld; // Angular velocity in world frame
    Eigen::Vector3d domegaWorld; // Angular acceleration in world frame

    std::array<Eigen::Vector3d, 4> ddqJoint; // Joint acceleration

    std::array<Eigen::Vector3d, 4> pFoot; // Foot position wrt hip
    std::array<Eigen::Vector3d, 4> vFoot; // Foot velocity wrt hip
    std::array<Eigen::Vector3d, 4> pFootBody; // Foot position wrt body
    std::array<Eigen::Vector3d, 4> vFootBody; // Foot velocity wrt body (vFootBody = vFoot)

};

struct DesiredStates {
    Eigen::Vector3d rpy_des;
    Eigen::Vector3d pos_des;

    //Desired states in world frame
    Eigen::Vector3d vWorld_des;
    Eigen::Vector3d aWorld_des;
    Eigen::Vector3d omegaWorld_des;
    Eigen::Vector3d domegaWorld_des;
    std::array<Eigen::Vector3d, 4> pFootWorld_des;
    std::array<Eigen::Vector3d, 4> vFootWorld_des;
    std::array<Eigen::Vector3d, 4> aFootWorld_des;

    //Desired states in body frame
    Eigen::Vector3d vBody_des;
    Eigen::Vector3d aBody_des;
    Eigen::Vector3d omegaBody_des;
    Eigen::Vector3d domegaBody_des;
    std::array<Eigen::Vector3d, 4> pFoot_des;
    std::array<Eigen::Vector3d, 4> vFoot_des;
    std::array<Eigen::Vector3d, 4> aFoot_des;

    //Desired contact states
    std::array<int, 4> contactStates;
};

#endif