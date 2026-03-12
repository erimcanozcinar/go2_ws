#ifndef FOOT_POSITION_TASK_HPP
#define FOOT_POSITION_TASK_HPP

#include "legged_controllers/WBC/TaskSet/Task.hpp"


class FootPosTask : public Task {
    public:
    FootPosTask(Robot* _model=nullptr, RobotStates* _lowStates=nullptr, EstimatorData* _estData=nullptr, int _legID=0);
    ~FootPosTask();

    protected:
    virtual void calcTask(const Eigen::VectorXd& x_des, const Eigen::VectorXd& dx_des, 
        const Eigen::VectorXd& ddx_des);
    virtual void calcTaskJacobian();

    private:
    Robot* model;
    RobotStates* lowStates;
    EstimatorData* estData;
    
    int legID;
    Eigen::Matrix3d Jac;
    Eigen::Matrix3d rSkew;
};

#endif