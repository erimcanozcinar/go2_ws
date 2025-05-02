#ifndef SIMULATION_HPP
#define SIMULATION_HPP

#include "raisim/World.hpp"
#include "raisim/RaisimServer.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"


class simulation {
    private:
    double period = 0.001;
    double period_ns = period*1e9;   

    raisim::ArticulatedSystem *robot;
    Eigen::VectorXd initialConditions;
    Eigen::Vector3d Fcon_LF, Fcon_RF, Fcon_LB, Fcon_RB;
    Eigen::Vector3d Pcon_LF, Pcon_RF, Pcon_LB, Pcon_RB;
    Eigen::VectorXd jointTorques;  

    void contactDefinition();
    
    public:
    double t, dt;

    simulation();
    ~simulation();
    void init();
    void run();


};

#endif