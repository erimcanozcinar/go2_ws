#ifndef BALANCECONTROLLER_HPP
#define BALANCECONTROLLER_HPP

// #include "functions.hpp"
#include "eigen3/Eigen/Dense"
#include <qpOASES.hpp>
#include <iostream>
#include "legged_common/matTools.hpp"
#include "legged_common/DataTypes.hpp"

#define GRAVITY 9.81
#define MASS 15.098

using namespace qpOASES;

static const int NUM_CONTACT_POINTS = 4;
static const int NUM_VARIABLES_PER_FOOT = 3;
static const double NEGATIVE_NUMBER = -1000000.0;
static const double POSITIVE_NUMBER = 1000000.0;


class BalanceController {
    public:
    std::array<Eigen::Vector3d, 4> footForce;

    BalanceController(EstimatorData* _estData = nullptr);
    ~BalanceController();

    void run();
    void setDesiredStates(DesiredStates* _desStates);
    void setActualStates();
    const std::array<Eigen::Vector3d, 4>& getFootForces() const { return footForce; }
    
    private:
    EstimatorData* _est;

    Eigen::Matrix3d Itorso, II;
    Eigen::Matrix3d Rz_act;
    std::array<int, 4> contactStates;
    std::array<Eigen::Vector3d, 4> pCom2Foot;
    Eigen::Vector3d gravity;
    Eigen::VectorXd F,prevF;

    Eigen::Vector3d rpy_des, rpy_act, rpy_int, rpy_comp;
    Eigen::Matrix3d rBody, rBody_des;
    Eigen::Vector3d pos_error, vel_error, orientation_error, angVel_error;
    Eigen::Vector3d acc_Com_world_des, omegadot_Com_world_des;
    Eigen::Vector3d pCOM, vCOM, omega, pCOM_des, vCOM_des, omega_des;


    /* Eigen Variables that Match qpOASES variables */
    Eigen::MatrixXd A_control;
    Eigen::VectorXd b_control;
    
    /* QP cost function weights */
    Eigen::MatrixXd S_control;  // Weight matrix for (A_control*F-b_control)^T*S_control*(A_control*F-b_control)
    Eigen::MatrixXd W_control;  // Weight matrix for regularization term F^T*W_control*F
    double alpha;               // Regularization weight for ||F||^2
    double beta;                 // Smoothing weight for ||F-prevF||^2
    
    /* QP formulation variables (Eigen) */
    Eigen::MatrixXd H_eigen;  // Hessian: 2*(A_control^T*S_control*A_control + alpha*W_control + beta*W_control)
    Eigen::MatrixXd g_eigen;  // Gradient: -2*A_control^T*S_control*b_control - 2*beta*W_control*prevF
    
    /* Constraint variables */
    Eigen::MatrixXd C_eigen;  // Constraint matrix: 20x12 (5 constraints per contact point * 4 contact points)
    Eigen::VectorXd lbC_eigen;  // Lower constraint bounds: 20x1
    Eigen::VectorXd ubC_eigen;  // Upper constraint bounds: 20x1
    Eigen::Vector3d n;          // Normal vector
    Eigen::Vector3d tx;         // Tangential direction x
    Eigen::Vector3d ty;         // Tangential direction y
    double mu;                  // Friction coefficient
    double fmin;                // Minimum normal force
    double fmax;                // Maximum normal force

    real_t cpu_time_fixed = 0.002;
    real_t cpu_time;
    
    /* qpOASES arrays (row-major format) */
    real_t H_qp[12 * 12];
    real_t g_qp[12];
    real_t lb_qp[12];   // Lower bounds for 12 variables
    real_t ub_qp[12];   // Upper bounds for 12 variables
    real_t C_qp[20 * 12];  // Constraint matrix: 20x12
    real_t lbC_qp[20];     // Lower constraint bounds: 20x1
    real_t ubC_qp[20];     // Upper constraint bounds: 20x1
    real_t xOpt[12];
    real_t yOpt[12 + 6];  // solution + dual variables

    void compensationRollPitch();
    void calcBodyPD();
    void update_A_control();
    void matrixLogRot(const Eigen::Matrix3d& R, Eigen::Vector3d& omega);
    void copy_Eigen_to_real_t(real_t* target, Eigen::MatrixXd& source, int nRows, int nCols);
    void calc_H_qp();
    void calc_g_qp();
    void calc_C_qp();
    void calc_lb_ub_qp();
    void solveQP_nonThreaded();

    /* qpOASES QP solver */
    SQProblem qpSolver;  // 12 variables, 20 constraints (5 per contact point * 4 contact points)

};

#endif