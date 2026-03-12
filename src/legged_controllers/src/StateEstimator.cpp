#include "legged_controllers/StateEstimator.hpp"

BasePosVelEstimator::BasePosVelEstimator(double dT) : dt(dT) {
    // Initialize state vector x
    x.setZero();
    x(2) = 0.3;
    y_hat.setZero();

    // Initialize covariance matrix P
    P = Eigen::Matrix<double, 18, 18>::Identity() * 1e-3;  // Small initial uncertainty

    // Initialize process noise covariance matrix Q
    Q = Eigen::Matrix<double, 18, 18>::Identity();
    Q_pos = Eigen::Matrix<double, 3, 3>::Identity() * 0.02;  // Small process noise for position
    Q_vel = Eigen::Matrix<double, 3, 3>::Identity() * 0.02;  // Small process noise for velocity
    Q_foot_pos = Eigen::Matrix<double, 3, 3>::Identity() *0.002;  // Small process noise for foot positions
    
    Q.block<3, 3>(0, 0) = Q_pos;
    Q.block<3, 3>(3, 3) = Q_vel;

    // Initialize measurement noise covariance matrix Rn
    Rn = Eigen::Matrix<double, 28, 28>::Identity();
    R_kin = Eigen::Matrix3d::Identity() * 0.001;  // Measurement noise for kinematics
    R_kin_vel = Eigen::Matrix3d::Identity() *0.1;  // Measurement noise for kinematics
    R_height =0.001;  // Measurement noise for height
    highScale = 100;


    // Initialize state transition matrix A
    A.setIdentity();
    A.block<3, 3>(0, 3) = dt * Eigen::Matrix3d::Identity();  // Position to velocity

    // Initialize control input matrix B
    B.setZero();
    B.block<3, 3>(0, 0) = 0.5 * dt * dt * Eigen::Matrix3d::Identity();
    B.block<3, 3>(3, 0) = dt * Eigen::Matrix3d::Identity();
    gravity<<0,0,-9.81;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
    C1 << Eigen::Matrix<double, 3, 3>::Identity(), Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
    C2 << Eigen::Matrix<double, 3, 3>::Zero(), Eigen::Matrix<double, 3, 3>::Identity();
    C.setZero();
    C.block(0, 0, 3, 6) = C1;
    C.block(3, 0, 3, 6) = C1;
    C.block(6, 0, 3, 6) = C1;
    C.block(9, 0, 3, 6) = C1;
    C.block(0, 6, 12, 12) = -1 * Eigen::Matrix<double, 12, 12>::Identity();
    C.block(12, 0, 3, 6) = C2;
    C.block(15, 0, 3, 6) = C2;
    C.block(18, 0, 3, 6) = C2;
    C.block(21, 0, 3, 6) = C2;
    C(27, 17) = 1;
    C(26, 14) = 1;
    C(25, 11) = 1;
    C(24, 8) = 1;
    Ctranspose=C.transpose();
    Atranspose=A.transpose();
    contact_prob.setZero();

    eta = 20;
}

void BasePosVelEstimator::predict(const Eigen::Vector3d &imu_acc, 
                               const Eigen::Matrix3d &imu_rWorld, 
                               const std::array<int, 4> &contact_states) {

    Eigen::Vector3d u = imu_rWorld*imu_acc+gravity;
    // Eigen::Vector3d u = imu_acc;
    // Add realistic high frequency noise to imu_acc



    x = A * x + B * u;

    // **Low-Pass Probability**
    double alpha = 0.9;
    for (int i = 0; i < 4; ++i) {
         contact_prob(i) = alpha * contact_prob(i) + (1.0 - alpha) * contact_states[i];

        double Q_scale = 1.0 + (1.0 - contact_prob(i)) * highScale; 
 
        // 
        //  double Q_scale = exp((1.0 - contact_prob(i)) * highScale);
        Q.block<3, 3>(6 + 3 * i, 6 + 3 * i) = Q_foot_pos * Q_scale;
    }

    P = A * P * Atranspose + Q;
}


void BasePosVelEstimator::updateWithKinematics(const std::array<Eigen::Vector3d, 4>& foot_pos, const std::array<Eigen::Vector3d, 4>& foot_vel) {
    // Calculate residual
    for (int i = 0; i < 4; ++i) {
        y_hat.segment<3>(i * 3) << -foot_pos[i] - (x.segment<3>(0) - x.segment<3>(6 + i * 3));
        y_hat.segment<3>(12 + i * 3) << contact_prob(i)*(-foot_vel[i]) - x.segment<3>(3);
        y_hat(24 + i) = - (x(8 + 3 * i))*1 ;
    }


    // Adaptively tune measurement noise covariance with contact states
    for (int i = 0; i < 4; ++i) { 
        double R_scale = 1.0 + (1.0 - contact_prob(i)) * highScale;
        Rn.block<3, 3>(3 * i, 3 * i) = R_kin * R_scale;
        Rn.block<3, 3>(12 + 3 * i, 12 + 3 * i) = R_kin_vel * R_scale;
        Rn(24 + i, 24 + i) = R_height * R_scale;
    }
    // Innovation covariance
    Eigen::MatrixXd S = C * P * Ctranspose + Rn;

    // Update states
    Eigen::MatrixXd S_ey = S.lu().solve(y_hat);
    x += P * Ctranspose * S_ey;  // Güncellenmiş state

    // Update solution
    Eigen::MatrixXd S_C = S.lu().solve(C);
    P = (Eigen::MatrixXd::Identity(18, 18) - P * Ctranspose * S_C) * P;

    // Do symmetric form
    Eigen::MatrixXd Pt = P.transpose();
    P = (P + Pt) / 2.0;

    // Stabilization of P
    if (P.block(0, 0, 2, 2).determinant() > 1e-6) {
        P.block(0, 2, 2, 16).setZero();
        P.block(2, 0, 16, 2).setZero();
        P.block(0, 0, 2, 2) /= 10.0;
    }

}

BaseOrientationEstimator::BaseOrientationEstimator(double dT) : dt(dT) {
    
}

void BaseOrientationEstimator::predict(const Eigen::Vector3d& imu_gyro, const Quat& imu_orientation) {
    q = imu_orientation;
    rWorld2Body = quaternionToRotationMatrix(q);
    eulerAngles = quatToRPY(q);
    gyroBody = imu_gyro;
    gyroWorld = rWorld2Body.transpose()*gyroBody;

    // Unwrap euler angles
    for(int i = 0; i<3; i++) {
        dgyroBody(i) = Numdiff(gyroBody(i), prev_gyroBody(i), dt);
        eulerAngles_uw(i) = funUnwrap(eulerAngles(i), prev_eulerAngles(i), cumSumdeltaCorr[i]);
    }
    prev_eulerAngles = eulerAngles;
    prev_gyroBody = gyroBody;

    dgyroWorld = rWorld2Body.transpose()*dgyroBody;
}



Estimator::Estimator(Robot* model, double dT) : dt(dT), _model(model) {
    std::cout << "Sampling rate of state estimator: " << 1/dt << " Hz" << std::endl;
    basePosVelEstimator = BasePosVelEstimator(dt);
    baseOrientationEstimator = BaseOrientationEstimator(dt);
}

Quat Estimator::orientationOffset(const Quat& q_imu) {
    if(first){
        Eigen::Vector3d rpy_ini = quatToRPY(q_imu);
        rpy_ini[0] = 0;
        rpy_ini[1] = 0;
        _ori_ini_inv = rpyToQuat(-rpy_ini);
        first = false;
    }
    return quatProduct(_ori_ini_inv, q_imu);
}

void Estimator::run(const RobotStates& _lowStates, const std::array<int, 4> &contact_states) {
    calcForwardKinematics(_model, _lowStates);

    baseOrientationEstimator.predict(_lowStates.imu.gyro, _lowStates.imu.orientation);
    basePosVelEstimator.updateWithKinematics(pFoot2Body_alignedWorldFrame, vFoot2Body_alignedWorldFrame);
    basePosVelEstimator.predict(_lowStates.imu.rotationMatrix.transpose()*_lowStates.imu.acc, 
                                _lowStates.imu.rotationMatrix.transpose(), 
                                contact_states);

    result.orientation = baseOrientationEstimator.getOrientation();
    result.rpy = baseOrientationEstimator.getRPY();
    result.rWorld2Body = baseOrientationEstimator.getRotMatrix();
    result.rBody2World = result.rWorld2Body.transpose();
    result.pos = basePosVelEstimator.getBasePosition();
    
    
    result.vWorld = basePosVelEstimator.getBaseVelocity();
    result.aWorld = result.rBody2World * _lowStates.imu.acc;
    result.omegaWorld = baseOrientationEstimator.getAngularVelocityWorld();
    result.domegaWorld = baseOrientationEstimator.getAngularAccelerationWorld();

    result.vBody = result.rWorld2Body * result.vWorld;
    result.aBody = _lowStates.imu.acc;
    result.omegaBody = baseOrientationEstimator.getAngularVelocityBody();
    result.domegaBody = baseOrientationEstimator.getAngularAccelerationBody();

    result.ddqJoint = ddqJoint;
    
    result.pFoot = pFoot2Hip;
    result.vFoot = vFoot2Hip;
    result.pFootBody = pFoot2Body;
    result.vFootBody = vFoot2Body;
}

void Estimator::calcForwardKinematics(Robot* _model, const RobotStates& _lowStates) {
    for(int i = 0; i < 3; i++) {
        ddqJoint[0](i) = Numdiff(_lowStates.dqJoint[0](i), prev_dqJoint[0](i), dt);
        ddqJoint[1](i) = Numdiff(_lowStates.dqJoint[1](i), prev_dqJoint[1](i), dt);
        ddqJoint[2](i) = Numdiff(_lowStates.dqJoint[2](i), prev_dqJoint[2](i), dt);
        ddqJoint[3](i) = Numdiff(_lowStates.dqJoint[3](i), prev_dqJoint[3](i), dt);
    }
    prev_dqJoint[0] = _lowStates.dqJoint[0];
    prev_dqJoint[1] = _lowStates.dqJoint[1];
    prev_dqJoint[2] = _lowStates.dqJoint[2];
    prev_dqJoint[3] = _lowStates.dqJoint[3];


    pFoot2Hip[0] = _model->_RF.calcFootPos(_lowStates.qJoint[0], FrameType::HIP);
    pFoot2Hip[1] = _model->_LF.calcFootPos(_lowStates.qJoint[1], FrameType::HIP);
    pFoot2Hip[2] = _model->_RB.calcFootPos(_lowStates.qJoint[2], FrameType::HIP);
    pFoot2Hip[3] = _model->_LB.calcFootPos(_lowStates.qJoint[3], FrameType::HIP);
    vFoot2Hip[0] = _model->_RF.calcFootVel(_lowStates.qJoint[0], _lowStates.dqJoint[0]);
    vFoot2Hip[1] = _model->_LF.calcFootVel(_lowStates.qJoint[1], _lowStates.dqJoint[1]);
    vFoot2Hip[2] = _model->_RB.calcFootVel(_lowStates.qJoint[2], _lowStates.dqJoint[2]);
    vFoot2Hip[3] = _model->_LB.calcFootVel(_lowStates.qJoint[3], _lowStates.dqJoint[3]);

    pFoot2Body[0] = _model->_RF.calcFootPos(_lowStates.qJoint[0], FrameType::BASE);
    pFoot2Body[1] = _model->_LF.calcFootPos(_lowStates.qJoint[1], FrameType::BASE);
    pFoot2Body[2] = _model->_RB.calcFootPos(_lowStates.qJoint[2], FrameType::BASE);
    pFoot2Body[3] = _model->_LB.calcFootPos(_lowStates.qJoint[3], FrameType::BASE);
    vFoot2Body = vFoot2Hip;

    pFoot2Body_alignedWorldFrame[0] = _lowStates.imu.rotationMatrix.transpose()*pFoot2Body[0];
    pFoot2Body_alignedWorldFrame[1] = _lowStates.imu.rotationMatrix.transpose()*pFoot2Body[1];
    pFoot2Body_alignedWorldFrame[2] = _lowStates.imu.rotationMatrix.transpose()*pFoot2Body[2];
    pFoot2Body_alignedWorldFrame[3] = _lowStates.imu.rotationMatrix.transpose()*pFoot2Body[3];
    vFoot2Body_alignedWorldFrame[0] = _lowStates.imu.rotationMatrix.transpose()*(_lowStates.imu.gyro.cross(pFoot2Body[0]) + vFoot2Hip[0]);
    vFoot2Body_alignedWorldFrame[1] = _lowStates.imu.rotationMatrix.transpose()*(_lowStates.imu.gyro.cross(pFoot2Body[1]) + vFoot2Hip[1]);
    vFoot2Body_alignedWorldFrame[2] = _lowStates.imu.rotationMatrix.transpose()*(_lowStates.imu.gyro.cross(pFoot2Body[2]) + vFoot2Hip[2]);
    vFoot2Body_alignedWorldFrame[3] = _lowStates.imu.rotationMatrix.transpose()*(_lowStates.imu.gyro.cross(pFoot2Body[3]) + vFoot2Hip[3]);
}

Estimator::~Estimator() {
    // delete basePosVelEstimator;
}