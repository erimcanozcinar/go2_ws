#include "legged_common/kinematics.hpp"

Leg::Leg(int leg){
    switch (leg)
    {
    case 1:
        _xSign = 1;
        _ySign = 1;
        break;
    case 2:
        _xSign = 1;
        _ySign = -1;
        break;
    case 3:
        _xSign = -1;
        _ySign = 1;
        break;
    case 4:
        _xSign = -1;
        _ySign = -1;
        break;
    }
    // qJoint << 0, (45*M_PI/180), (-90*M_PI/180);
    pHip2Body << _xSign*0.1934, _ySign*0.0465, 0.0;
    _abadLinkLength = 0.0955;
    _thighLinkLength = 0.213;
    _calfLinkLength = 0.213; // 0.213 + 0.02 (link length + foot radius)
    pHipAA = pHip2Body;
    pHipFE << 0, _ySign*_abadLinkLength, 0;
    pKneeFE << 0, 0, -_thighLinkLength;
    pFootEnd << 0, 0, -_calfLinkLength;
    footFrameBody = pHip2Body + pHipFE; // each foot placement on ground(xy) plane
    footPosNormalStand << 0, _ySign*_abadLinkLength, -initZc;
    footPosBodyNormalStand = pHip2Body + footPosNormalStand;
}

Eigen::Vector3d Leg::calcFootPos(const Eigen::Vector3d& Q, FrameType frame) {
    // Link Lengths
    Eigen::Vector4d vone;
    vone << 0, 0, 0, 1; 

    Eigen::Matrix4d T01, T12, T23, T34, T45;
    Eigen::Matrix4d T02, T03, T04, T05;

    if(frame == FrameType::HIP) {
        pHipAA = Eigen::Vector3d::Zero();
    } else if(frame == FrameType::BASE) {
        pHipAA = pHip2Body;
    }
    
    T01 << RotateRoll(Q(0)), pHipAA, vone.transpose();
    T12 << RotatePitch(Q(1)), pHipFE, vone.transpose();
    T23 << RotatePitch(Q(2)), pKneeFE, vone.transpose();
    T34 << Eigen::MatrixXd::Identity(3,3), pFootEnd, vone.transpose();

    // T01 = (T01);
    // T02 = (T01*T12);
    // T03 = (T01*T12*T23);
    T04 = (T01*T12*T23*T34);

    // footPos(0) = T04(0,3);
    // footPos(1) = T04(1,3);
    // footPos(2) = T04(2,3);
    return T04.block<3,1>(0,3);
}

Eigen::Vector3d Leg::calcFootVel(const Eigen::Vector3d& Q, const Eigen::Vector3d& dQ) {
    return calcLegJac(Q) * dQ;
}

Eigen::Matrix3d Leg::calcLegJac(const Eigen::Vector3d& q) {
    Eigen::Matrix3d signMat;
    Eigen::Matrix3d Jac;
    signMat << _xSign, 0, 0,
               0, _ySign, 0,
               0, 0, _ySign;
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);

    double L2 = _ySign*_abadLinkLength;
    double L3 = -_thighLinkLength;
    double Lf = -_calfLinkLength;

    Jac(0,0) = 0.0;
    Jac(0,1) = (Lf*cos(q2 + q3) + L3*cos(q2));
    Jac(0,2) = (Lf*cos(q2 + q3));

    Jac(1,0) = (- L2*sin(q1) - Lf*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - L3*cos(q1)*cos(q2));
    Jac(1,1) = (sin(q1)*(Lf*sin(q2 + q3) + L3*sin(q2)));
    Jac(1,2) = (Lf*sin(q2 + q3)*sin(q1));

    Jac(2,0) = (Lf*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + L2*cos(q1) - L3*cos(q2)*sin(q1));
    Jac(2,1) = (-cos(q1)*(Lf*sin(q2 + q3) + L3*sin(q2)));
    Jac(2,2) = (-Lf*sin(q2 + q3)*cos(q1));

    // Jac = Jac*signMat;
    return Jac;
}

Eigen::Matrix3d Leg::calcLegJacDot(const Eigen::Vector3d& q, const Eigen::Vector3d& dq) {
    Eigen::Matrix3d signMat;
    Eigen::Matrix3d dJac;
    signMat << _xSign, 0, 0,
               0, _ySign, 0,
               0, 0, _ySign;
    double q1 = q(0);
    double q2 = q(1);
    double q3 = q(2);
    double dq1 = dq(0);
    double dq2 = dq(1);
    double dq3 = dq(2);

    double L2 = _ySign*_abadLinkLength;
    double L3 = -_thighLinkLength;
    double Lf = -_calfLinkLength;

    dJac(0,0) = 0.0;
    dJac(0,1) = (- dq2*(Lf*sin(q2 + q3) + L3*sin(q2)) - Lf*dq3*sin(q2 + q3));
    dJac(0,2) = (-Lf*sin(q2 + q3)*(dq2 + dq3));

    dJac(1,0) = (dq2*(Lf*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + L3*cos(q1)*sin(q2)) - dq1*(Lf*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + L2*cos(q1) - L3*cos(q2)*sin(q1)) + Lf*dq3*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)));
    dJac(1,1) = (dq2*sin(q1)*(Lf*cos(q2 + q3) + L3*cos(q2)) + dq1*cos(q1)*(Lf*sin(q2 + q3) + L3*sin(q2)) + Lf*dq3*cos(q2 + q3)*sin(q1));
    dJac(1,2) = (Lf*dq1*sin(q2 + q3)*cos(q1) + Lf*dq2*cos(q2 + q3)*sin(q1) + Lf*dq3*cos(q2 + q3)*sin(q1));

    dJac(2,0) = (dq2*sin(q1)*(Lf*sin(q2 + q3) + L3*sin(q2)) - dq1*(L2*sin(q1) + L3*cos(q1)*cos(q2) + Lf*cos(q1)*cos(q2)*cos(q3) - Lf*cos(q1)*sin(q2)*sin(q3)) + Lf*dq3*sin(q2 + q3)*sin(q1));
    dJac(2,1) = (dq1*sin(q1)*(Lf*sin(q2 + q3) + L3*sin(q2)) - dq2*cos(q1)*(Lf*cos(q2 + q3) + L3*cos(q2)) - Lf*dq3*cos(q2 + q3)*cos(q1));
    dJac(2,2) = (Lf*dq1*sin(q2 + q3)*sin(q1) - Lf*dq3*cos(q2 + q3)*cos(q1) - Lf*dq2*cos(q2 + q3)*cos(q1));

    return dJac;
}

Eigen::Vector3d Leg::calcJointPos(const Eigen::Vector3d& pfoot){ 
    double L2 = _ySign*_abadLinkLength;
    double L3 = -_thighLinkLength;
    double Lf = -_calfLinkLength;

    // Knee joint angle
    double Cosq3 = (pfoot(0)*pfoot(0) + pfoot(1)*pfoot(1) + pfoot(2)*pfoot(2) - L2*L2 - L3*L3 - Lf*Lf)/(2*L3*Lf);
    double Sinq3 = -sqrt(1-Cosq3*Cosq3);
    double q3 = atan2(Sinq3, Cosq3);

    // Thigh joint angle
    double A = -sqrt(pfoot(1)*pfoot(1) + pfoot(2)*pfoot(2) - L2*L2);
    double a = L3 + Lf*cos(q3);
    double b = Lf*sin(q3);
    double Cosq2 = (pfoot(0)*b + A*a)/(a*a + b*b);
    double Sinq2 = (pfoot(0)*a - A*b)/(a*a + b*b);
    double q2 = atan2(Sinq2,Cosq2);
    
    // Hip joint angle
    double An = Lf*cos(q2+q3) + L3*cos(q2);
    double Cosq1 = (pfoot(1)*L2 + pfoot(2)*An)/(L2*L2 + An*An);
    double Sinq1 = (pfoot(2)*L2 - pfoot(1)*An)/(L2*L2 + An*An);
    double q1 = atan2(Sinq1,Cosq1);

    return Eigen::Vector3d(q1, q2, q3);
}

Eigen::Vector3d Leg::calcJointVel(const Eigen::Vector3d& vfoot, const Eigen::Vector3d& q){
    return calcLegJac(q).inverse()*vfoot;
}



Robot::Robot() : _LF(1), _RF(2), _LB(3), _RB(4){
}

Eigen::Matrix3d Robot::calcBodyJac(const Eigen::Vector3d& rpy){
    Eigen::Matrix3d Jac;
    double p = rpy(1);
    double y = rpy(2);
    Jac << cos(rpy(1))*cos(rpy(2)), -sin(rpy(2)), 0,
           cos(rpy(1))*sin(rpy(2)),  cos(rpy(2)), 0,
           -sin(rpy(1)), 0, 1;
    return Jac;
}
