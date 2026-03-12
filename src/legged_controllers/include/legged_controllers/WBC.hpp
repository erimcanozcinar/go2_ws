#ifndef WHOLE_BODY_CONTROL_HPP
#define WHOLE_BODY_CONTROL_HPP

#include "eigen3/Eigen/Dense"
#include <vector>
#include "legged_common/kinematics.hpp"
#include "legged_common/parameters.hpp"
#include "legged_controllers/TaskSet/SingleContactTask.hpp"
#include "legged_controllers/TaskSet/BodyOriTask.hpp"
#include "legged_controllers/TaskSet/BodyPosTask.hpp"
#include "legged_controllers/TaskSet/FootPosTask.hpp"

class WBC {
    public:
    WBC(Robot* _model = nullptr, EstimatorData* _estData = nullptr, RobotStates* _lowStates = nullptr);
    ~WBC();

    Eigen::VectorXd getJointVelCmd() {return dq_cmd_result.tail(12);}
    Eigen::VectorXd getJointPosCmd() {return delta_q_result.tail(12);}
    void  prioritizedTaskExecution();
    void setDesiredStates(DesiredStates* _desiredStates);

    private:
    void resizeMatrices(int _taskDim);

    Robot* model;
    EstimatorData* estData;
    RobotStates* lowStates;

    Task* _bodyPosTask;
    Task* _bodyOriTask;
    std::array<Task*, 4> _footPosTasks;
    std::array<Task*, 4> _singleContactTasks;

    std::vector<Task*> _tasks, contactTasks;

    int dof = 18;
    Eigen::MatrixXd Nt, Nt_pre; // Null Space Matrix

    Eigen::MatrixXd Jc, Jc_pinv;
    Eigen::MatrixXd Jt;     // Task Jacobian
    Eigen::MatrixXd Jt_pre;      // Projected Jacobian (J * N_prev)
    Eigen::MatrixXd Jt_pre_pinv;  // Projected Jacobian's pseudo 
    Eigen::VectorXd task_error;
    Eigen::VectorXd dx_des;
    Eigen::VectorXd delta_q; // delta_q_cmd
    Eigen::VectorXd dq_cmd; // dq_cmd

    Eigen::VectorXd dq_cmd_result;
    Eigen::VectorXd delta_q_result;

    Eigen::VectorXd xDes, dxDes, ddxDes;
    std::array<int, 4> _contactStates;
};

#endif