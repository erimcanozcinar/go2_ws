#include "legged_trajectory/Trajectory.hpp"

Trajectory::Trajectory(EstimatorData* _estData, double _dT) : dT(_dT), comTraj(_dT), _est(_estData) {
    std::cout << "Sampling rate of trajectory generation: " << 1/dT << " Hz" << std::endl;

    jStick = &GamePad::getInstance();
    gait = new Gait(dT);

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

    _est->pos << 0, 0, initZc;

    pFoot = pFoot_initial;
    vFoot.fill(Eigen::Vector3d::Zero());
    vFootWorld.fill(Eigen::Vector3d::Zero());
    conState = {1,1,1,1};

    for(int i = 0; i<4; i++) { 
        footSwingTraj[i].setSwingTime(gait->getSwingPeriod());
        pFootWorld[i] = _est->pos + Eigen::Matrix3d::Identity()*(pHip[i] + pFoot[i]);
    }
    p0 = pFootWorld;

    pfx.setZero();
    pfy.setZero();
    pfz.setZero();
    
    Vcmd.setZero();
}

void Trajectory::trajGeneration() {   

    if(gait->getGaitType() != STAND) gait->run();
    gait->switchGait(jStick->gait);


    Vcmd = _est->rBody2World*jStick->vBody;
    Vcmd(2) = jStick->wBody(2);
    comTraj.zCom = _est->pos(2);

    comTraj.comTrajPlanner(Vcmd.topRows(2), _est->vWorld.head(2), jStick->zCom, gait->getStancePeriod());
    // Pcom_des = comTraj.getComPos();
    Vcom_des = comTraj.getComVel();
    Acom_des = comTraj.getComAcc();


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
                p0[i] = _est->pos + _est->rBody2World*(pHip[i] + _est->pFoot[i]);
            }
            p0[i] = p0[i];
        } else {
            comTraj.calcStride(Vcmd, _est->vWorld.head(2), gait->getTimeSwingRemaining(i), gait->getStancePeriod());
            // pRobotFrame[i](1) += 0.2*yShift[i]*jStick->vBody(0);
            pYawCorrected[i] = RotateYaw(jStick->wBody(2)*gait->getStancePeriod()/2)*pRobotFrame[i];
            pf[i] = _est->pos + _est->rBody2World*(pYawCorrected[i]) + comTraj.getStrideLength();
            // pf[i](2) = p0[i](2);
            Eigen::Vector3d p0_body = _est->rWorld2Body * (p0[i] - _est->pos);
            Eigen::Vector3d pf_body = _est->rWorld2Body * (pf[i] - _est->pos);
            pf_body(2) = p0_body(2);
            pf[i] = _est->pos + _est->rBody2World * pf_body;
            footSwingTraj[i].footStepPlanner(gait->getPhaseSwing(i), p0[i], pf[i], comTraj.getFootHeight());
            pFootWorld[i] = footSwingTraj[i].getFootPos();
            vFootWorld[i] = footSwingTraj[i].getFootVel();
            aFootWorld[i] = footSwingTraj[i].getFootAcc();
        }        

        pFoot[i] = _est->rWorld2Body*(pFootWorld[i] - _est->pos) - pHip[i];
        vFoot[i] = _est->rWorld2Body*(vFootWorld[i] - _est->vWorld);
        aFoot[i] = _est->rWorld2Body*(aFootWorld[i] - Acom_des);
    }

    Eigen::Vector2d estRP = slopeEstimation();

    _desiredStates.pos_des = Pcom_des;
    _desiredStates.rpy_des = Eigen::Vector3d(estRP(0)*0, estRP(1), _est->rpy(2)  + dT*jStick->wBody(2));

    _desiredStates.vWorld_des = Vcom_des;
    _desiredStates.aWorld_des = Acom_des;
    _desiredStates.omegaWorld_des = _est->rBody2World*Eigen::Vector3d(0, 0, jStick->wBody(2));
    _desiredStates.domegaWorld_des = Eigen::Vector3d::Zero();
    _desiredStates.pFootWorld_des = pFootWorld;
    _desiredStates.vFootWorld_des = vFootWorld;
    _desiredStates.aFootWorld_des = aFootWorld;

    _desiredStates.vBody_des = _est->rWorld2Body*Vcom_des;
    _desiredStates.aBody_des = _est->rWorld2Body*Acom_des;
    _desiredStates.omegaBody_des = Eigen::Vector3d(0, 0, jStick->wBody(2));
    _desiredStates.domegaBody_des = Eigen::Vector3d::Zero();
    _desiredStates.pFoot_des = pFoot;
    _desiredStates.vFoot_des = vFoot;
    _desiredStates.aFoot_des = aFoot;
    
    _desiredStates.contactStates = conState;
}

Eigen::Vector2d Trajectory::slopeEstimation() {
    std::array<Eigen::Vector3d, 4> pf;       
    Eigen::MatrixXd W(4,3);
    Eigen::Vector3d a;
    for(int i = 0; i < 4; i++) {
        pf[i] = RotateYaw(_est->rpy(2)).transpose()*(_est->pos + _est->rBody2World*_est->pFootBody[i]);
        if(conState[i] == 1) {
            pfx(i) = pf[i](0);
            pfy(i) = pf[i](1);
            pfz(i) = pf[i](2);
        }
    }
    W << Eigen::Vector4d::Ones(), pfx, pfy;
    a = (W.transpose()*W).inverse()*W.transpose()*pfz;
    // std::cout << "Roll: " << atan2(a(2),1) << " Pitch: " << atan2(-a(1),1) << std::endl;
    return Eigen::Vector2d(atan2(a(2),1), atan2(-a(1),1));
}
