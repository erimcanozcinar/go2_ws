#include "legged_controllers/WBC/WBIC.hpp"

WBIC::WBIC(Robot* _model, RigidBodyModel* _rigidBodyModel, EstimatorData* _estData, RobotStates* _lowStates) : model(_model), rbModel(_rigidBodyModel), estData(_estData), lowStates(_lowStates) {
    _bodyPosTask = new BodyPosTask(model, estData);
    _bodyOriTask = new BodyOriTask(model, estData);
    _footPosTasks[0] = new FootPosTask(model, lowStates, estData, 0);
    _footPosTasks[1] = new FootPosTask(model, lowStates, estData, 1);
    _footPosTasks[2] = new FootPosTask(model, lowStates, estData, 2);
    _footPosTasks[3] = new FootPosTask(model, lowStates, estData, 3);
    _singleContactTasks[0] = new SingleContactTask(model, lowStates, estData, 0);
    _singleContactTasks[1] = new SingleContactTask(model, lowStates, estData, 1);
    _singleContactTasks[2] = new SingleContactTask(model, lowStates, estData, 2);
    _singleContactTasks[3] = new SingleContactTask(model, lowStates, estData, 3);

    Nt.resize(dof, dof);
    Nt_pre.resize(dof, dof);
    Npre.resize(dof, dof);
    
    delta_q.resize(dof);
    dq_cmd.resize(dof); 
    ddq_cmd.resize(dof);
    
    dq_cmd_result.resize(dof);
    delta_q_result.resize(dof);
    dq_cmd_result.setZero();
    delta_q_result.setZero();
    ddq_cmd.setZero();

    genCoord.resize(dof+1);
    genVel.resize(dof);
    _M.resize(dof, dof);
    _Minv.resize(dof, dof);
    _coriolis.resize(dof);
    _gravity.resize(dof);
    tau_ff.resize(dof);
    tau_ff.setZero();

    _Jc.resize(12,18);
    Fr_des.resize(12,1);

    xDes.resize(dof+1);
    dxDes.resize(dof);
    ddxDes.resize(dof);

    G.resize(0.0,18,18);
    g0.resize(0.0,18);
    CE.resize(0.0,18,6);
    ce0.resize(0.0,6);
    CI.resize(0.0,18,24);
    ci0.resize(0.0,24);

    A_eq.resize(6,18);
    A_ineq.resize(24,18);
    B_eq.resize(6,1);
    B_ineq.resize(24,1);

    Sf.resize(6,18);
    Sf.setZero();
    Sf.block(0,0,6,6) = Eigen::MatrixXd::Identity(6,6);

    Uf.resize(6, 3);
    Uf.setZero();
    _Uf.resize(24, 12);
    _Uf.setZero();
    Uf_ieq_vec.resize(6,1);
    Uf_ieq_vec.setZero();
    _Uf_ieq_vec.resize(24,1);
    _Uf_ieq_vec.setZero();
    Uf << 0, 0, 1,
          1, 0, mu,
          -1,0, mu,
          0, 1, mu,
          0, -1, mu,
          0, 0, -1;
    Uf_ieq_vec << 0, 0, 0, 0, 0, -Fzmax;
    for(int i = 0; i < 4; ++i) {
        _Uf.block(6*i,3*i,6,3) = Uf;
        _Uf_ieq_vec.block(6*i,0,6,1) = Uf_ieq_vec;
    }

    Wf.resize(6, 1);
    Wf = Eigen::VectorXd::Ones(6)*0.1;
    Wfr.resize(12, 1);
    Wfr = Eigen::VectorXd::Ones(12);
}

WBIC::~WBIC() {
    delete _bodyOriTask;
    delete _bodyPosTask;
    for(int i = 0; i < 4; i++) {
        delete _footPosTasks[i];
        delete _singleContactTasks[i];
    }
    _tasks.clear();
}

void WBIC::prioritizedTaskExecution() {  
    _tasks.clear();
    contactTasks.clear();  

    // Update Tasks
    _bodyOriTask->updateTask(xDes.block(0,0,4,1), dxDes.block(0,0,3,1), ddxDes.block(0,0,3,1));
    _bodyPosTask->updateTask(xDes.block(4,0,3,1), dxDes.block(3,0,3,1), ddxDes.block(3,0,3,1));
    
    // Fill tasks list
    _tasks.push_back(_bodyOriTask);
    _tasks.push_back(_bodyPosTask);
    // Decide Contact and Swing Tasks    
    for(int i = 0; i < 4; ++i) {
        _singleContactTasks[i]->updateTask(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        if(_contactStates[i] > 0) { // Contact 
            contactTasks.push_back(_singleContactTasks[i]);     
        } else { // Swing
            _footPosTasks[i]->updateTask(xDes.block(3*i+7,0,3,1), dxDes.block(3*i+6,0,3,1), ddxDes.block(3*i+6,0,3,1));
            _tasks.push_back(_footPosTasks[i]);
        }
    }

    // Calculate Task 0: Contact Task Jacobian and Null Space Matrix
    Jc.resize(3*contactTasks.size(), dof);
    dJc.resize(3*contactTasks.size(), dof);
    Jc_pinv.resize(dof, 3*contactTasks.size());
    JcBar.resize(dof, 3*contactTasks.size());
    if(contactTasks.size() > 0) {
        for(int i = 0; i < contactTasks.size(); ++i) {
            Jc.block(3*i,0,3,dof) = contactTasks[i]->getTaskJacobian();
            dJc.block(3*i,0,3,dof) = contactTasks[i]->getTaskJacobianDot();
        }
        pseudoInverse(Jc, 0.0001, Jc_pinv);
        Nt_pre = Eigen::MatrixXd::Identity(dof, dof) - Jc_pinv * Jc; // Nt = N0
        weightedPseudoInverse(Jc, _Minv, JcBar);
        ddq_cmd = JcBar*(-dJc);
        Npre = Eigen::MatrixXd::Identity(dof, dof) - JcBar * Jc;
    } else {
        Nt_pre.setIdentity();
        ddq_cmd.setZero();
        Npre.setIdentity();
    }
    delta_q.setZero();
    dq_cmd.setZero(); 

    // Main Tasks Execution: Calculate Task i+1
    for(size_t i = 0; i < _tasks.size(); ++i) {
        resizeMatrices(_tasks[i]->getTaskDim());
        
        Jt = _tasks[i]->getTaskJacobian();
        dJt = _tasks[i]->getTaskJacobianDot();

        Jt_pre = Jt * Nt_pre;
        JtPre = Jt * Npre;
        pseudoInverse(Jt_pre, 0.0001, Jt_pre_pinv);
        weightedPseudoInverse(JtPre, _Minv, JtBar);
        delta_q += Jt_pre_pinv * (_tasks[i]->getError() - Jt * delta_q);
        dq_cmd += Jt_pre_pinv * (_tasks[i]->getDesVel() - Jt * dq_cmd);
        ddq_cmd += JtBar * (_tasks[i]->getCmdAcc() - dJt*genVel - Jt*ddq_cmd);


        Nt = (Eigen::MatrixXd::Identity(dof, dof) - Jt_pre_pinv * Jt_pre);
        Nt_pre *= Nt; 
    }

    Eigen::VectorXd q_act(18);
    q_act << 0, 0, 0, 0, 0, 0, lowStates->qJoint[0], lowStates->qJoint[1], lowStates->qJoint[2], lowStates->qJoint[3];
    dq_cmd_result = dq_cmd;
    delta_q_result = q_act + delta_q;    
}

void WBIC::setEqualityConstraint() {
    A_eq.block(0,0,6,6) = _M.block(0,0,6,6);
    A_eq.block(0,6,6,12) = -Sf*_Jc.transpose();
    B_eq = -Sf*(_M*ddq_cmd + _coriolis + _gravity - _Jc.transpose()*Fr_des);

    for(int r = 0; r < 6; ++r) {
        for(int c = 0; c < 18; ++c) {
            CE[c][r] = A_eq(r,c);
        }
        ce0[r] = -B_eq(r);
    }
}

void WBIC::setInequalityConstraint() {
    A_ineq.setZero();
    A_ineq.block(0,6,24,12) = _Uf;
    B_ineq = _Uf_ieq_vec - _Uf*Fr_des;
    for(int r = 0; r < 24; ++r) {
        for(int c = 0; c < 18; ++c) {
            CI[c][r] = A_ineq(r,c);
        }
        ci0[r] = -B_ineq(r);
    }
}

void WBIC::setCostFunction() {
    for(int r = 0; r < 18; ++r) {
        for(int c = 0; c < 18; ++c) {
            G[r][c] = 0.0;
        }
    }

    for(int i = 0; i < 6; ++i) {
        G[i][i] = Wf(i);
    }
    for(int i = 0; i < 12; ++i) {
        G[i+6][i+6] = Wfr(i);
    }
}
void WBIC::run(const std::array<Eigen::Vector3d, 4>& Fc_des) {
    Fr_des << Fc_des[0], Fc_des[1], Fc_des[2], Fc_des[3];
    updateDynamics();
    prioritizedTaskExecution();
    setEqualityConstraint();
    setInequalityConstraint();
    setCostFunction();

    double f = solve_quadprog(G, g0, CE, ce0, CI, ci0, z);

    for(int i = 0; i < 6; ++i) ddq_cmd(i) += z[i];
    Eigen::VectorXd _Fr(12);
    for(int i = 0; i < 12; i++) Fr_des(i) = z[i+6] + Fr_des(i);


    for(int i = 0; i < 4; i++) {
        _Jc.block(3*i,0,3,18) = _singleContactTasks[i]->getTaskJacobian();
    }
    tau_ff = _M*ddq_cmd + _coriolis + _gravity - _Jc.transpose() * Fr_des;
}

void WBIC::setDesiredStates(DesiredStates* _desiredStates) {
    Eigen::Matrix3d R_des = RotateYaw(_desiredStates->rpy_des(2))*RotatePitch(_desiredStates->rpy_des(1))*RotateRoll(_desiredStates->rpy_des(0));
    xDes.block(0,0,4,1) = rotationMatrixToQuaternion(R_des.transpose());
    xDes.block(4,0,3,1) = _desiredStates->pos_des;
    xDes.block(7,0,3,1) = _desiredStates->pFootWorld_des[0];
    xDes.block(10,0,3,1) = _desiredStates->pFootWorld_des[1];
    xDes.block(13,0,3,1) = _desiredStates->pFootWorld_des[2];
    xDes.block(16,0,3,1) = _desiredStates->pFootWorld_des[3];

    dxDes.block(0,0,3,1) = _desiredStates->omegaBody_des; 
    dxDes.block(3,0,3,1) = _desiredStates->vWorld_des;
    dxDes.block(6,0,3,1) = _desiredStates->vFootWorld_des[0];
    dxDes.block(9,0,3,1) = _desiredStates->vFootWorld_des[1];
    dxDes.block(12,0,3,1) = _desiredStates->vFootWorld_des[2];
    dxDes.block(15,0,3,1) = _desiredStates->vFootWorld_des[3];

    ddxDes.block(0,0,3,1) = _desiredStates->omegaBody_des;
    ddxDes.block(3,0,3,1) = _desiredStates->aWorld_des;
    ddxDes.block(6,0,3,1) = _desiredStates->aFootWorld_des[0];
    ddxDes.block(9,0,3,1) = _desiredStates->aFootWorld_des[1];
    ddxDes.block(12,0,3,1) = _desiredStates->aFootWorld_des[2];
    ddxDes.block(15,0,3,1) = _desiredStates->aFootWorld_des[3];

    _contactStates = _desiredStates->contactStates;
}

void WBIC::updateDynamics() {
    genCoord << estData->orientation, estData->pos, lowStates->qJoint[0], lowStates->qJoint[1], lowStates->qJoint[2], lowStates->qJoint[3];
    genVel << estData->omegaBody, estData->vBody, lowStates->dqJoint[0], lowStates->dqJoint[1], lowStates->dqJoint[2], lowStates->dqJoint[3];

    _M = rbModel->getMassMatrix(genCoord);
    _gravity = rbModel->getGravityForces(genCoord);
    _coriolis = rbModel->getCoriolisForces(genCoord, genVel);
    _Minv = _M.inverse();
}

void WBIC::resizeMatrices(int _taskDim) {
    Jt.resize(_taskDim, dof);
    Jt_pre.resize(_taskDim, dof);
    Jt_pre_pinv.resize(dof, _taskDim);    
    task_error.resize(_taskDim);
    dx_des.resize(_taskDim);

    dJt.resize(_taskDim, dof);
    JtBar.resize(dof, _taskDim);
    JtPre.resize(_taskDim, dof);
}
