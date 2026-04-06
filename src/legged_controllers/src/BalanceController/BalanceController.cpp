#include "legged_controllers/BalanceController/BalanceController.hpp"


BalanceController::BalanceController(EstimatorData* _estData) : qpSolver(12, 20), _est(_estData) {
    Rz_act.setIdentity();
    Itorso << 0.02448, 0, 0,
              0, 0.098077, 0,
              0, 0, 0.107;

    gravity << 0, 0, -GRAVITY;
    contactStates.fill(1);

    A_control.resize(6, 12);
    A_control.setZero();
    b_control.resize(6);
    b_control.setZero();
    F.resize(12); prevF.resize(12);
    F.setZero(); prevF.setZero();
    F(2) = 30.0;
    F(5) = 30.0;
    F(8) = 30.0;
    F(11) = 30.0;
    prevF = F;  

    // Initialize QP cost function weights
    S_control.resize(6, 6);
    S_control.setIdentity();  // Default: identity matrix
    W_control.resize(12, 12);
    W_control.setIdentity();  // Default: identity matrix
    Eigen::VectorXd s(6), w(12);
    s << 1, 1, 10, 20, 10, 10;
    w << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    S_control = s.asDiagonal();
    W_control = w.asDiagonal();
    alpha = 0.01;  // Default regularization weight for ||F||^2
    beta = 0.01;   // Default smoothing weight for ||F-prevF||^2

    // Initialize constraint variables
    C_eigen.resize(20, 12);
    C_eigen.setZero();
    lbC_eigen.resize(20);
    ubC_eigen.resize(20);
    n << 0, 0, 1;  // Default: z-axis normal (upward)
    tx << 1, 0, 0; // Default: x-axis tangential
    ty << 0, 1, 0; // Default: y-axis tangential
    mu = 0.5;      // Default friction coefficient
    fmin = 0.0;    // Default minimum normal force
    fmax = 160.0;  // Default maximum normal force

    // Initialize QP variables (Eigen)
    H_eigen.resize(12, 12);
    H_eigen.setZero();
    g_eigen.resize(12, 1);
    g_eigen.setZero();

    // Initialize qpOASES arrays (fixed-size arrays are automatically allocated)
    // Arrays are initialized to zero by default

    // Initialize qpOASES solver options
    Options options;
    options.printLevel = PL_NONE;
    qpSolver.setOptions(options);

}

BalanceController::~BalanceController() {
}

void BalanceController::run() {
    setActualStates();
    calcBodyPD();    
    update_A_control();
    solveQP_nonThreaded();
    
    for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
        footForce[i] = -rBody.transpose()*F.segment(3 * i, 3);
        footForceWorld[i] = F.segment(3 * i, 3);
    }

}

void BalanceController::calcBodyPD() {
    pos_error = Rz_act.transpose()*(pCOM_des - pCOM);
    vel_error = Rz_act.transpose()*(vCOM_des - vCOM);

    compensationRollPitch();
    matrixLogRot(Rz_act.transpose()*rBody_des*rBody.transpose()*Rz_act, orientation_error);
    angVel_error = Rz_act.transpose()*(omega_des - omega);

    acc_Com_world_des(0) = 70*pos_error(0) + 10*vel_error(0);
    acc_Com_world_des(1) = 70*pos_error(1) + 10*vel_error(1);
    acc_Com_world_des(2) = 70*pos_error(2) + 10*vel_error(2);
    omegadot_Com_world_des(0) = 780*orientation_error(0) + 70*angVel_error(0);
    omegadot_Com_world_des(1) = 780*orientation_error(1) + 70*angVel_error(1);
    omegadot_Com_world_des(2) = 780*orientation_error(2) + 70*angVel_error(2);

    II = Rz_act.transpose()*rBody.transpose()*Itorso*rBody*Rz_act;

    b_control << MASS*(acc_Com_world_des - gravity), II*omegadot_Com_world_des;
}

void BalanceController::setDesiredStates(DesiredStates* _desStates) {
    pCOM_des = _desStates->pos_des;
    vCOM_des = _desStates->vWorld_des;
    omega_des = _desStates->omegaWorld_des;
    rpy_des = _desStates->rpy_des;
    contactStates = _desStates->contactStates;
}

void BalanceController::setActualStates() {
    pCOM = _est->pos;
    vCOM = _est->vWorld;
    omega = _est->omegaWorld;
    rBody = _est->rBody2World;
    rpy_act = _est->rpy;
    Rz_act = RotateYaw(_est->rpy(2)); // Rotate yaw body to world frame
    pCom2Foot[0] = _est->rBody2World*_est->pFootBody[0];
    pCom2Foot[1] = _est->rBody2World*_est->pFootBody[1];
    pCom2Foot[2] = _est->rBody2World*_est->pFootBody[2];
    pCom2Foot[3] = _est->rBody2World*_est->pFootBody[3];
}

void BalanceController::update_A_control() {
    for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
        A_control.block(0, 3*i, 3, 3) = contactStates[i]*Rz_act.transpose();      
        A_control.block(3, 3*i, 3, 3) = Rz_act.transpose()*vec2SkewSym(contactStates[i]*pCom2Foot[i]);
    }
}

void BalanceController::calc_H_qp() {
    H_eigen = 2.0 * (A_control.transpose() * S_control * A_control + alpha * W_control + beta * W_control);    
    // Copy Eigen matrix to qpOASES array (row-major format)
    copy_Eigen_to_real_t(H_qp, H_eigen, 12, 12);
}

void BalanceController::calc_g_qp() {
    g_eigen = -2.0 * A_control.transpose() * S_control * b_control; // VectorXd automatically converts to MatrixXd (12x1)
    g_eigen += -2.0 * beta * W_control * prevF;  // Smoothing term: -2*beta*W_control*prevF
    // Copy Eigen matrix to qpOASES array (row-major format)
    copy_Eigen_to_real_t(g_qp, g_eigen, 12, 1);
}

void BalanceController::calc_C_qp() {
    // Build constraint matrix C: 20x12 (5 constraints per contact point * 4 contact points)
    // Constraint: lbC < C*F < ubC
    // c matrix (5x3) for each contact point:
    // c = [(-mu*n + tx)^T
    //      (-mu*n + ty)^T
    //      (+mu*n + tx)^T
    //      (+mu*n + ty)^T
    //      n^T]
    
    C_eigen.setZero();
    
    // Build c matrix (5x3)
    Eigen::MatrixXd c(5, 3);
    c.row(0) = (-mu * n + tx).transpose();
    c.row(1) = (-mu * n + ty).transpose();
    c.row(2) = (+mu * n + tx).transpose();
    c.row(3) = (+mu * n + ty).transpose();
    c.row(4) = n.transpose();
    
    // Place c matrix for each contact point (block diagonal structure)
    for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
        C_eigen.block(5*i, 3*i, 5, 3) = c;
    }
    
    // Build constraint bounds
    // lb = [-100000 -100000 0 0 fmin]^T (repeated for each contact point)
    // ub = [0 0 100000 100000 fmax]^T (repeated for each contact point)
    for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
        lbC_eigen(5 * i + 0) = contactStates[i]*NEGATIVE_NUMBER;
        lbC_eigen(5 * i + 1) = contactStates[i]*NEGATIVE_NUMBER;
        lbC_eigen(5 * i + 2) = 0.0;
        lbC_eigen(5 * i + 3) = 0.0;
        lbC_eigen(5 * i + 4) = contactStates[i]*fmin;
        
        ubC_eigen(5 * i + 0) = 0.0;
        ubC_eigen(5 * i + 1) = 0.0;
        ubC_eigen(5 * i + 2) = contactStates[i]*POSITIVE_NUMBER;
        ubC_eigen(5 * i + 3) = contactStates[i]*POSITIVE_NUMBER;
        ubC_eigen(5 * i + 4) = contactStates[i]*fmax;
    }
    
    // Copy Eigen matrices to qpOASES arrays (row-major format)
    copy_Eigen_to_real_t(C_qp, C_eigen, 20, 12);
    for (int i = 0; i < 20; i++) {
        lbC_qp[i] = lbC_eigen(i);
        ubC_qp[i] = ubC_eigen(i);
    }
}

void BalanceController::calc_lb_ub_qp() {
    // Set variable bounds: lb < F < ub
    // For each contact point, set bounds based on contact state
    // If contact is active (contactStates[i] == 1), allow forces in range [NEGATIVE_NUMBER, POSITIVE_NUMBER]
    // If contact is inactive (contactStates[i] == 0), set bounds to 0 (no force)
    for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
        for (int j = 0; j < NUM_VARIABLES_PER_FOOT; j++) {
            lb_qp[NUM_VARIABLES_PER_FOOT * i + j] = contactStates[i] * NEGATIVE_NUMBER;
            ub_qp[NUM_VARIABLES_PER_FOOT * i + j] = contactStates[i] * POSITIVE_NUMBER;
        }
    }
}

void BalanceController::solveQP_nonThreaded() {
    calc_H_qp();
    calc_g_qp();
    calc_C_qp();
    calc_lb_ub_qp();

    // Solve QP problem with constraints and variable bounds
    int nWSR = 100; // maximum number of working set recalculations
    cpu_time = cpu_time_fixed;
    returnValue qpStatus;
    if(!qpSolver.isInitialised()) {
        qpStatus = qpSolver.init(H_qp, g_qp, C_qp, lb_qp, ub_qp, lbC_qp, ubC_qp, nWSR, &cpu_time);
    } else {
        qpStatus = qpSolver.hotstart(H_qp, g_qp, C_qp, lb_qp, ub_qp, lbC_qp, ubC_qp, nWSR, &cpu_time);
    }
    if(qpStatus != SUCCESSFUL_RETURN) {
        std::cout << "QP solver failed with status: " << qpStatus << std::endl;
    }
    // Get solution
    qpSolver.getPrimalSolution(xOpt);

    // Copy solution back to Eigen vector
    for (int i = 0; i < 12; i++) {
        F(i) = xOpt[i];
    }
    prevF = F;
}

void BalanceController::compensationRollPitch() {
    if(std::fabs(vCOM(1)) > 0.1) rpy_int(0) += 0.002*(rpy_des(0) - rpy_act(0))/vCOM(1);
    if(std::fabs(vCOM(0)) > 0.2) rpy_int(1) += 0.002*(rpy_des(1) - rpy_act(1))/vCOM(0);
    rpy_int(0) = fminf(fmaxf(rpy_int(0), -.25), .25);
    rpy_int(1) = fminf(fmaxf(rpy_int(1), -.25), .25);

    rpy_comp(0) = vCOM(1)*rpy_int(0);
    rpy_comp(1) = vCOM(0)*rpy_int(1);
    rpy_comp(2) = rpy_des(2);
    rBody_des = RotateYaw(rpy_comp(2))*RotatePitch(rpy_comp(1))*RotateRoll(rpy_comp(0));
}

void BalanceController::matrixLogRot(const Eigen::Matrix3d& R, Eigen::Vector3d& omega) {
    double theta;
    double tmp = (R(0, 0) + R(1, 1) + R(2, 2) - 1) / 2;
    if (tmp >= 1.) {
        theta = 0;
    } else if (tmp <= -1.) {
        theta = M_PI;
    } else {
        theta = acos(tmp);
    }

    // Matrix3F omegaHat = (R-R.transpose())/(2 * sin(theta));
    // crossExtract(omegaHat,omega);
    omega << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
    if (theta > 10e-5) {
        omega *= theta / (2 * sin(theta));
    } else {
        omega /= 2;
    }
}

void BalanceController::copy_Eigen_to_real_t(real_t* target, Eigen::MatrixXd& source, int nRows, int nCols) {
    // Copy Eigen matrix to qpOASES real_t array in row-major format
    // qpOASES expects matrices stored row by row: [row0_col0, row0_col1, ..., row0_colN, row1_col0, ...]
    int count = 0;
    for (int i = 0; i < nRows; i++) {
        for (int j = 0; j < nCols; j++) {
            target[count] = source(i, j);
            count++;
        }
    }
}
