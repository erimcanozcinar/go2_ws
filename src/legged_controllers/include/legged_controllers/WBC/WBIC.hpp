#ifndef WHOLE_BODY_IMPULSE_CONTROL_HPP
#define WHOLE_BODY_IMPULSE_CONTROL_HPP

#include "eigen3/Eigen/Dense"
#include "legged_common/kinematics.hpp"
#include "legged_common/parameters.hpp"
#include "rbdyn/RigidBodyModel.hpp"
#include <QuadProg++.hh>
#include "legged_controllers/WBC/TaskSet/SingleContactTask.hpp"
#include "legged_controllers/WBC/TaskSet/BodyOriTask.hpp"
#include "legged_controllers/WBC/TaskSet/BodyPosTask.hpp"
#include "legged_controllers/WBC/TaskSet/FootPosTask.hpp"

class WBIC {
    public:
    WBIC(Robot* _model = nullptr, RigidBodyModel* _rigidBodyModel = nullptr, EstimatorData* _estData = nullptr, RobotStates* _lowStates = nullptr);
    ~WBIC();

    Eigen::VectorXd getJointVelCmd() {return dq_cmd_result.tail(12);}
    Eigen::VectorXd getJointPosCmd() {return delta_q_result.tail(12);}
    Eigen::VectorXd getTauFF() {return tau_ff.tail(12);}
    void setDesiredStates(DesiredStates* _desiredStates);
    void run(const std::array<Eigen::Vector3d, 4>& Fc_des);

    private:
    void prioritizedTaskExecution();
    void setEqualityConstraint();
    void setInequalityConstraint();
    void setCostFunction();
    void updateDynamics();
    void resizeMatrices(int _taskDim);

    Robot* model;
    RigidBodyModel* rbModel;
    EstimatorData* estData;
    RobotStates* lowStates;

    Task* _bodyPosTask;
    Task* _bodyOriTask;
    std::array<Task*, 4> _footPosTasks;
    std::array<Task*, 4> _singleContactTasks;

    std::vector<Task*> _tasks, contactTasks;

    int dof = 18;
    Eigen::MatrixXd Nt, Nt_pre, Npre; // Null Space Matrix

    Eigen::MatrixXd Jc, Jc_pinv, dJc, JcBar;
    Eigen::MatrixXd Jt, dJt, JtBar;     // Task Jacobian
    Eigen::MatrixXd Jt_pre, JtPre;      // Projected Jacobian (J * N_prev)
    Eigen::MatrixXd Jt_pre_pinv;  // Projected Jacobian's pseudo 
    Eigen::VectorXd task_error;
    Eigen::VectorXd dx_des;
    Eigen::VectorXd delta_q; // delta_q_cmd
    Eigen::VectorXd dq_cmd, ddq_cmd; // dq_cmd and ddq_cmd

    Eigen::VectorXd dq_cmd_result;
    Eigen::VectorXd delta_q_result;

    Eigen::VectorXd genCoord, genVel;
    Eigen::MatrixXd _M, _Minv;
    Eigen::VectorXd _coriolis, _gravity;
    Eigen::VectorXd tau_ff;

    Eigen::MatrixXd _Jc, Fr_des;

    Eigen::VectorXd xDes, dxDes, ddxDes;
    std::array<int, 4> _contactStates;

    // QP Variables
    GolDIdnani::GVect<double> z;
    GolDIdnani::GMatr<double> G;
    GolDIdnani::GVect<double> g0;
    GolDIdnani::GMatr<double> CE;
    GolDIdnani::GVect<double> ce0;
    GolDIdnani::GMatr<double> CI;
    GolDIdnani::GVect<double> ci0;

    // Eigen Variables for QP
    Eigen::MatrixXd A_eq, A_ineq;
    Eigen::MatrixXd B_eq, B_ineq;

    Eigen::MatrixXd Sf;
    double mu = 0.5; // Friction coefficient
    double Fzmax = 80.0; // Maximum normal force
    Eigen::MatrixXd Uf, _Uf; // Friction cone
    Eigen::MatrixXd Uf_ieq_vec, _Uf_ieq_vec;
    Eigen::VectorXd Wf, Wfr;
};

#endif