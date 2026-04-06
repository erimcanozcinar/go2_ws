#include "legged_controllers/WBC/TaskSet/BodyPosTask.hpp"

BodyPosTask::BodyPosTask(Robot* _model, EstimatorData* _estData) 
            : Task(3, "BodyPosTask"), model(_model), estData(_estData) {
    TK::Jt = Eigen::MatrixXd::Zero(TK::taskDim, TK::dof);
    TK::dJt = Eigen::MatrixXd::Zero(TK::taskDim, TK::dof);
    TK::error = Eigen::VectorXd::Zero(TK::taskDim);
    TK::desVel = Eigen::VectorXd::Zero(TK::taskDim);

    TK::ddx_cmd = Eigen::VectorXd::Zero(TK::taskDim);
}

BodyPosTask::~BodyPosTask() {
}

void BodyPosTask::calcTask(const Eigen::VectorXd& x_des, const Eigen::VectorXd& dx_des, 
    const Eigen::VectorXd& ddx_des) {
        TK::error = x_des - estData->pos;
        TK::desVel = dx_des;

        TK::ddx_cmd = ddx_des + 100*TK::error + 10*(TK::desVel - estData->vWorld);
}

void BodyPosTask::calcTaskJacobian() {
    // Identity matrix is used because desired body velocity is in world frame
    TK::Jt.block(0,3,3,3) = Eigen::Matrix3d::Identity();
}

void BodyPosTask::calcTaskJacobianDot() {
    TK::dJt.block(0,3,3,3) = Eigen::Matrix3d::Zero();
}
