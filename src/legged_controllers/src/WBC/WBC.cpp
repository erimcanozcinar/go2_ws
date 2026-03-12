#include "legged_controllers/WBC/WBC.hpp"

WBC::WBC(Robot* _model, EstimatorData* _estData, RobotStates* _lowStates) : model(_model), estData(_estData), lowStates(_lowStates) {
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
    
    delta_q.resize(dof);
    dq_cmd.resize(dof); 
    
    dq_cmd_result.resize(dof);
    delta_q_result.resize(dof);
    dq_cmd_result.setZero();
    delta_q_result.setZero();

    xDes.resize(dof+1);
    dxDes.resize(dof);
    ddxDes.resize(dof);
}

WBC::~WBC() {
    delete _bodyOriTask;
    delete _bodyPosTask;
    for(int i = 0; i < 4; i++) {
        delete _footPosTasks[i];
        delete _singleContactTasks[i];
    }
    _tasks.clear();
}

void WBC::prioritizedTaskExecution() {  
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
        if(_contactStates[i] > 0) { // Contact 
            _singleContactTasks[i]->updateTask(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
            contactTasks.push_back(_singleContactTasks[i]);     
        } else { // Swing
            _footPosTasks[i]->updateTask(xDes.block(3*i+7,0,3,1), dxDes.block(3*i+6,0,3,1), ddxDes.block(3*i+6,0,3,1));
            _tasks.push_back(_footPosTasks[i]);
        }
        // _footPosTasks[i]->updateTask(xDes.block(3*i+7,0,3,1), dxDes.block(3*i+6,0,3,1), ddxDes.block(3*i+6,0,3,1));
        // _tasks.push_back(_footPosTasks[i]);
    }

    // Calculate Task 0: Contact Task Jacobian and Null Space Matrix
    Jc.resize(3*contactTasks.size(), dof);
    Jc_pinv.resize(dof, 3*contactTasks.size());
    if(contactTasks.size() > 0) {
        for(int i = 0; i < contactTasks.size(); ++i) {
            Jc.block(3*i,0,3,dof) = contactTasks[i]->getTaskJacobian();
        }
        pseudoInverse(Jc, 0.0001, Jc_pinv);
        Nt_pre = Eigen::MatrixXd::Identity(dof, dof) - Jc_pinv * Jc; // Nt = N0
    } else {
        Nt_pre.setIdentity();
    }
    delta_q.setZero();
    dq_cmd.setZero(); 

    // Main Tasks Execution: Calculate Task i+1
    for(size_t i = 0; i < _tasks.size(); ++i) {
        resizeMatrices(_tasks[i]->getTaskDim());
        
        Jt = _tasks[i]->getTaskJacobian();
        Jt_pre = Jt * Nt_pre;
        pseudoInverse(Jt_pre, 0.0001, Jt_pre_pinv);
        delta_q += Jt_pre_pinv * (_tasks[i]->getError() - Jt * delta_q);
        dq_cmd += Jt_pre_pinv * (_tasks[i]->getDesVel() - Jt * dq_cmd);


        Nt = (Eigen::MatrixXd::Identity(dof, dof) - Jt_pre_pinv * Jt_pre);
        Nt_pre *= Nt; 
    }
    dq_cmd_result = dq_cmd;
    delta_q_result = delta_q;
    
}


void WBC::setDesiredStates(DesiredStates* _desiredStates) {
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

    ddxDes.setZero();

    _contactStates = _desiredStates->contactStates;
}

void WBC::resizeMatrices(int _taskDim) {
    Jt.resize(_taskDim, dof);
    Jt_pre.resize(_taskDim, dof);
    Jt_pre_pinv.resize(dof, _taskDim);    
    task_error.resize(_taskDim);
    dx_des.resize(_taskDim);
}
