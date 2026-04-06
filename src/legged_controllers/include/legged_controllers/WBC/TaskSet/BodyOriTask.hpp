#ifndef BODY_ORIENTATION_TASK_HPP
#define BODY_ORIENTATION_TASK_HPP

#include "legged_controllers/WBC/TaskSet/Task.hpp"

class BodyOriTask : public Task {
    public:
    BodyOriTask(Robot* _model, EstimatorData* _estData);
    ~BodyOriTask();

    protected:
    virtual void calcTask(const Eigen::VectorXd& x_des, const Eigen::VectorXd& dx_des, 
        const Eigen::VectorXd& ddx_des);
    virtual void calcTaskJacobian();
    virtual void calcTaskJacobianDot();

    private:
    Robot* model;
    EstimatorData* estData;
};

#endif