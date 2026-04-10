#include "legged_trajectory/Trajectory.hpp"

Trajectory::Trajectory(EstimatorData* _estData, double _dT) : dT(_dT), comTraj(_dT), _est(_estData) {
    jStick.intiSDL2();
    std::cout << "Sampling rate of trajectory generation: " << 1/dT << " Hz" << std::endl;


    gait = new Gait(dT);

    cmdJoyF[5] = initZc;
    pre_cmdJoyF[5] = initZc;

    pHip[0] = _RF.pHip2Body;
    pHip[1] = _LF.pHip2Body;
    pHip[2] = _RB.pHip2Body;
    pHip[3] = _LB.pHip2Body;

    pFoot_initial[0] = _RF.footPosNormalStand;
    pFoot_initial[1] = _LF.footPosNormalStand;
    pFoot_initial[2] = _RB.footPosNormalStand;
    pFoot_initial[3] = _LB.footPosNormalStand;

    pRobotFrame[0] = _RF.footFrameBody;
    pRobotFrame[1] = _LF.footFrameBody;
    pRobotFrame[2] = _RB.footFrameBody;
    pRobotFrame[3] = _LB.footFrameBody;

    Pcom << 0, 0, initZc;

    pFoot = pFoot_initial;
    vFoot.fill(Eigen::Vector3d::Zero());
    vFootWorld.fill(Eigen::Vector3d::Zero());
    conState = {1,1,1,1};

    for(int i = 0; i<4; i++) { 
        footSwingTraj[i].setSwingTime(gait->getSwingPeriod());
        pFootWorld[i] = Pcom + Eigen::Matrix3d::Identity()*(pHip[i] + pFoot[i]);
    }
    p0 = pFootWorld;
    
    Vcmd.setZero();
}

void Trajectory::trajGeneration() {   

    jStick.callGamePad();
    for(int i=0; i<22; i++)
    {
        cmdJoyF[i] = LPF(jStick.joyCmd[i], pre_cmdJoyF[i], 2*M_PI*0.2, dT);
        pre_cmdJoyF[i] = cmdJoyF[i];
    }

    if(gait->getGaitType() != STAND) gait->run();

    dYaw_des = cmdJoyF[2];
    // Yaw_des = _est->rpy(2);
    Yaw_des += cmdJoyF[2]*dT;

    Vcmd = _est->rBody2World*Eigen::Vector3d(cmdJoyF[0], cmdJoyF[1], 0);
    Vcmd(2) = cmdJoyF[2];
    comTraj.zCom = _est->pos(2);

    comTraj.comTrajPlanner(Vcmd.topRows(2), _est->vWorld.head(2), cmdJoyF[5], gait->getStancePeriod());
    // Pcom_des = comTraj.getComPos();
    Vcom_des = comTraj.getComVel();
    Acom_des = comTraj.getComAcc();

    gait->switchGait(jStick.gait);

    if(gait->getGaitType() != STAND) Pcom_des = _est->pos;
    else Pcom_des = Pcom_des;
    Pcom_des(2) = comTraj.getComPos()(2);

    pRobotFrame[0] = _RF.footFrameBody;
    pRobotFrame[1] = _LF.footFrameBody;
    pRobotFrame[2] = _RB.footFrameBody;
    pRobotFrame[3] = _LB.footFrameBody;
    for(int i = 0; i<4; i++) {
        footSwingTraj[i].setSwingTime(gait->getSwingPeriod());
        conState[i] = gait->getContactState(i);   
        
        if(conState[i] == 1){
            if(gait->getPhase(i) < 0.5) {
                p0[i] = Pcom + _est->rBody2World*(pHip[i] + _est->pFoot[i]);
            }
            p0[i] = p0[i];
        } else {
            comTraj.calcStride(Vcmd, _est->vWorld.head(2), gait->getTimeSwingRemaining(i), gait->getStancePeriod());
            pRobotFrame[i](1) += 0.2*yShift[i]*cmdJoyF[0];
            pYawCorrected[i] = RotateYaw(cmdJoyF[2]*gait->getStancePeriod()/2)*pRobotFrame[i];
            pf[i] = _est->pos + _est->rBody2World*(pYawCorrected[i]) + comTraj.getStrideLength();
            pf[i](2) = 0.0;
            footSwingTraj[i].footStepPlanner(gait->getPhaseSwing(i), p0[i], pf[i], comTraj.getFootHeight());
            pFootWorld[i] = footSwingTraj[i].getFootPos();
            vFootWorld[i] = footSwingTraj[i].getFootVel();
            aFootWorld[i] = footSwingTraj[i].getFootAcc();
        }        

        pFoot[i] = _est->rWorld2Body*(pFootWorld[i] - _est->pos) - pHip[i];
        vFoot[i] = _est->rWorld2Body*(vFootWorld[i] - _est->vWorld);
        aFoot[i] = _est->rWorld2Body*(aFootWorld[i] - Acom_des);
    }
    Pcom = _est->pos;

    _desiredStates.pos_des = Pcom_des;
    _desiredStates.rpy_des = Eigen::Vector3d(0, 0, Yaw_des);

    _desiredStates.vWorld_des = Vcom_des;
    _desiredStates.aWorld_des = Acom_des;
    _desiredStates.omegaWorld_des = Eigen::Vector3d(0, 0, dYaw_des);
    _desiredStates.domegaWorld_des = Eigen::Vector3d::Zero();
    _desiredStates.pFootWorld_des = pFootWorld;
    _desiredStates.vFootWorld_des = vFootWorld;
    _desiredStates.aFootWorld_des = aFootWorld;

    _desiredStates.vBody_des = _est->rWorld2Body*Vcom_des;
    _desiredStates.aBody_des = _est->rWorld2Body*Acom_des;
    _desiredStates.omegaBody_des = Eigen::Vector3d(0, 0, dYaw_des);
    _desiredStates.domegaBody_des = Eigen::Vector3d::Zero();
    _desiredStates.pFoot_des = pFoot;
    _desiredStates.vFoot_des = vFoot;
    _desiredStates.aFoot_des = aFoot;
    
    _desiredStates.contactStates = conState;
}