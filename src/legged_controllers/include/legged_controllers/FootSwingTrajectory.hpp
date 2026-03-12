#ifndef FOOTSWIINGTRAJECTORY_HPP
#define FOOTSWIINGTRAJECTORY_HPP

#include <eigen3/Eigen/Dense>

class FootSwingTrajectory {
    private:
        Eigen::Vector3d Pf, Vf, Af;
        Eigen::Vector3d trajX, trajY, trajZ;
        double t, tSwing;

        Eigen::Vector3d FuncPoly5th(double RealTime, double t_start, double t_end, double Z0, double dZ0, double ddZ0, double Ze, double dZe, double ddZe);
        Eigen::Vector3d FuncPoly6th(double RealTime, double t_start, double t_end, double Z0, double dZ0, double ddZ0, double Ze, double dZe, double ddZe, double Fh);

    public:
        FootSwingTrajectory(){};
        void footStepPlanner(double phaseSwg, Eigen::Vector3d p0, Eigen::Vector3d pf, double Fh);
        Eigen::Vector3d getFootPos() const { return Pf; }
        Eigen::Vector3d getFootVel() const { return Vf; }
        Eigen::Vector3d getFootAcc() const { return Af; }
        void setSwingTime(double tSw) { tSwing = tSw; }
        double getTime() const { return t; }
};


#endif