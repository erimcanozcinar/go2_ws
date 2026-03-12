#ifndef STATEESTIMATOR_HPP
#define STATEESTIMATOR_HPP

#include "iostream"
#include "legged_common/matTools.hpp"
#include "legged_common/DataTypes.hpp"
#include "legged_common/kinematics.hpp"

class BasePosVelEstimator {
    public:
    BasePosVelEstimator(double dT = 0.001);
    void predict(const Eigen::Vector3d& imu_acc, const RotMat& imu_rWorld, const std::array<int, 4>& contact_states);
    void updateWithKinematics(const std::array<Eigen::Vector3d, 4>& foot_pos, const std::array<Eigen::Vector3d, 4>& foot_vel);
    Eigen::Vector3d getBasePosition() const { return x.segment<3>(0); };
    Eigen::Vector3d getBaseVelocity() const { return x.segment<3>(3); };

    private:
    double dt;
    Eigen::Matrix<double, 18, 1> x;       // State vector [base position; base velocity; foot positions,foot Velocities]
    Eigen::Matrix<double, 28, 1> y_hat;    // Measurement vector
    Eigen::Matrix<double, 18, 18> P;      // State covariance
    Eigen::Matrix<double, 18, 18> Q;      // Process noise covariance
    Eigen::Matrix<double, 28, 28> Rn;         // Measurement noise covariance
    Eigen::Matrix<double,3,3> Q_pos;         // Measurement noise covariance
    Eigen::Matrix<double,3,3> Q_vel;         // Measurement noise covariance
    Eigen::Matrix<double,3,3> Q_foot_pos;         // Measurement noise covariance
    Eigen::Matrix<double,3,3> Q_foot_vel;         // Measurement noise covariance
    Eigen::Vector4d contact_prob;
    Eigen::Vector3d gravity{};
    Eigen::Matrix3d R_kin;         // Measurement noise covariance
    Eigen::Matrix3d R_kin_vel;         // Measurement noise covariance

    
    double R_height;         // Measurement noise covariance
    Eigen::Matrix<double, 18, 18> A;      // State transition matrix
    Eigen::Matrix<double, 18, 18> Atranspose;      // State transition matrix transpose
    double highScale;
    double eta;
    Eigen::Matrix<double, 18, 3> B;       // Input matrix
    Eigen::Matrix<double, 28, 18> C;      // Measurement matrix
    Eigen::Matrix<double, 18, 28> Ctranspose;      // Measurement matrix    
};

class BaseOrientationEstimator {
    public:
    BaseOrientationEstimator(double dT = 0.001);
    void predict(const Eigen::Vector3d& imu_gyro, const Quat& imu_orientation);
    Quat getOrientation() const { return q; };
    RotMat getRotMatrix() const { return rWorld2Body; };
    Eigen::Vector3d getRPY() const { return eulerAngles_uw; };
    Eigen::Vector3d getAngularVelocityBody() const { return gyroBody; };
    Eigen::Vector3d getAngularVelocityWorld() const { return gyroWorld; };
    Eigen::Vector3d getAngularAccelerationBody() const { return dgyroBody; };
    Eigen::Vector3d getAngularAccelerationWorld() const { return dgyroWorld; };

    private:
    double dt;
    Quat q;       // Orientation (quaternion)
    RotMat rWorld2Body; // Rotation matrix from body to world
    Eigen::Vector3d eulerAngles; // Roll, pitch, yaw angles
    Eigen::Vector3d eulerAngles_uw; // Unwraped roll, pitch, yaw angles
    Eigen::Vector3d gyroBody; // Angular velocity in body frame
    Eigen::Vector3d gyroWorld; // Angular velocity in world frame
    Eigen::Vector3d dgyroBody; // Angular acceleration in body frame
    Eigen::Vector3d dgyroWorld; // Angular acceleration in world frame
    Eigen::Vector3d prev_gyroBody; // Previous angular velocity in body frame

    Eigen::Vector3d prev_eulerAngles; // Roll, pitch, yaw angles
    std::array<int, 3> cumSumdeltaCorr = {0, 0, 0};

};



class Estimator {
    public:
    BasePosVelEstimator basePosVelEstimator;
    BaseOrientationEstimator baseOrientationEstimator;
    EstimatorData result;
    
    Estimator(Robot* model, double dT = 0.001);
    ~Estimator();
    Quat orientationOffset(const Quat& q_imu);
    void run(const RobotStates& _lowStates, const std::array<int, 4>& contact_states);
    EstimatorData getResult() const { return result; };

    
    private:
    double dt;
    Robot* _model;

    bool first = true;
    Quat _ori_ini_inv;    

    std::array<Eigen::Vector3d, 4> ddqJoint;
    std::array<Eigen::Vector3d, 4> prev_dqJoint;
    
    std::array<Eigen::Vector3d, 4> pFoot2Hip, vFoot2Hip;
    std::array<Eigen::Vector3d, 4> pFoot2Body, vFoot2Body;
    std::array<Eigen::Vector3d, 4> pFoot2Body_alignedWorldFrame, vFoot2Body_alignedWorldFrame;

    void calcForwardKinematics(Robot* _model, const RobotStates& _lowStates);
};

#endif





