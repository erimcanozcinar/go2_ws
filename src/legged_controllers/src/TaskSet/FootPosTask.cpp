#include "legged_controllers/TaskSet/FootPosTask.hpp"

FootPosTask::FootPosTask(Robot* _model, RobotStates* _lowStates, EstimatorData* _estData, int _legID) 
            : Task(3, "FootPosTask"), model(_model), lowStates(_lowStates), estData(_estData), legID(_legID) {
    TK::Jt = Eigen::MatrixXd::Zero(TK::taskDim, TK::dof);
    TK::error = Eigen::VectorXd::Zero(TK::taskDim);
    TK::desVel = Eigen::VectorXd::Zero(TK::taskDim);
    Jac.setZero();
}

FootPosTask::~FootPosTask() {
}

void FootPosTask::calcTask(const Eigen::VectorXd& x_des, const Eigen::VectorXd& dx_des, 
    const Eigen::VectorXd& ddx_des) {
        TK::error = (x_des-estData->pos) - (estData->rBody2World*estData->pFootBody[legID]);
        TK::desVel = dx_des;
}

void FootPosTask::calcTaskJacobian() {
        if(legID == 0) Jac = model->_RF.calcLegJac(lowStates->qJoint[0]);
        else if(legID == 1) Jac = model->_LF.calcLegJac(lowStates->qJoint[1]);
        else if(legID == 2) Jac = model->_RB.calcLegJac(lowStates->qJoint[2]);
        else if(legID == 3) Jac = model->_LB.calcLegJac(lowStates->qJoint[3]);
        else throw std::invalid_argument("Invalid leg ID");
        
        // Base angular part: In world frame
        rSkew = vec2SkewSym(estData->pFootBody[legID]);
        TK::Jt.block(0,0,3,3) = -estData->rBody2World*rSkew;
        // Base linear part: In world frame
        TK::Jt.block(0,3,3,3) = Eigen::Matrix3d::Identity();     
        // Foot position part: In world frame
        TK::Jt.block(0,6+3*legID,3,3) = estData->rBody2World*Jac;
}
