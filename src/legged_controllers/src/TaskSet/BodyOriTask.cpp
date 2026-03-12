#include "legged_controllers/TaskSet/BodyOriTask.hpp"

BodyOriTask::BodyOriTask(Robot* _model, EstimatorData* _estData) 
            : Task(3, "BodyOriTask"), model(_model), estData(_estData) {
    TK::Jt = Eigen::MatrixXd::Zero(TK::taskDim, TK::dof);
    TK::error = Eigen::VectorXd::Zero(TK::taskDim);
    TK::desVel = Eigen::VectorXd::Zero(TK::taskDim);
}

BodyOriTask::~BodyOriTask() {
}

void BodyOriTask::calcTask(const Eigen::VectorXd& x_des, const Eigen::VectorXd& dx_des, 
    const Eigen::VectorXd& ddx_des) {
        Quat quat_actual = estData->orientation;
        quat_actual(0) = quat_actual(0);
        quat_actual(1) = -quat_actual(1);
        quat_actual(2) = -quat_actual(2);
        quat_actual(3) = -quat_actual(3);
        Quat quat_error = quatProduct(x_des, quat_actual);
        if(quat_error(0) < 0.) {
            quat_error *= -1.;
        }
        Eigen::Vector3d so3_error;
        so3_error = quaternionToso3(quat_error);
        TK::error = so3_error;
        TK::desVel = dx_des;    
}

void BodyOriTask::calcTaskJacobian() {
    // Rbody2world is used because desiredOmega is in body frame
    TK::Jt.block(0,0,3,3) = estData->rBody2World;
}