#ifndef BODY_POSITION_TASK_HPP
#define BODY_POSITION_TASK_HPP

#include "legged_controllers/WBC/TaskSet/Task.hpp"

class BodyPosTask : public Task {
    public:
    BodyPosTask(Robot* _model, EstimatorData* _estData);
    ~BodyPosTask();

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