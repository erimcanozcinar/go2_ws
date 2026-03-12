#ifndef TASK_HPP
#define TASK_HPP

#include "eigen3/Eigen/Dense"
#include "legged_common/kinematics.hpp"
#include "legged_common/parameters.hpp"
#include "legged_estimator/StateEstimator.hpp"
#include "legged_common/DataTypes.hpp"


#define TK Task

class Task {
public:
    Task(int _taskDim=0, std::string _name="") : taskDim(_taskDim), name(_name) {}
    virtual ~Task() {}

    Eigen::MatrixXd getTaskJacobian() {return Jt;}
    Eigen::VectorXd getError() {return error;}
    Eigen::VectorXd getDesVel() {return desVel;}
    std::string getName() {return name;}
    int getTaskDim() {return taskDim;}
    void updateTask(const Eigen::VectorXd& x_des, const Eigen::VectorXd& dx_des, 
        const Eigen::VectorXd& ddx_des) {
        calcTask(x_des, dx_des, ddx_des);
        calcTaskJacobian();
    }
    
protected:
    virtual void calcTask(const Eigen::VectorXd& x_des, const Eigen::VectorXd& dx_des, 
        const Eigen::VectorXd& ddx_des) = 0;
    virtual void calcTaskJacobian() = 0;

    std::string name;
    int taskDim;
    int dof = 18;
    Eigen::MatrixXd Jt;
    Eigen::VectorXd error;
    Eigen::VectorXd desVel;
};

#endif