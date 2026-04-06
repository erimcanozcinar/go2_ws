#ifndef SINGLE_CONTACT_TASK_HPP
#define SINGLE_CONTACT_TASK_HPP

#include "legged_controllers/WBC/TaskSet/Task.hpp"

class SingleContactTask : public Task {
    public:
    SingleContactTask(Robot* _model, RobotStates* _lowStates, EstimatorData* _estData, int _legID);
    ~SingleContactTask();

    protected:
    virtual void calcTask(const Eigen::VectorXd& x_des, const Eigen::VectorXd& dx_des, 
        const Eigen::VectorXd& ddx_des);
    virtual void calcTaskJacobian();
    virtual void calcTaskJacobianDot();

    private:
    Robot* model;
    RobotStates* lowStates;
    EstimatorData* estData;

    int legID;
    Eigen::Matrix3d Jac, dJac;
    Eigen::Matrix3d rSkew, vSkew;
};

#endif