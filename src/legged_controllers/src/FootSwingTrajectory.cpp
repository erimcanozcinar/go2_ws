#include "legged_controllers/FootSwingTrajectory.hpp"

void FootSwingTrajectory::footStepPlanner(double phaseSwg, Eigen::Vector3d p0, Eigen::Vector3d pf, double Fh)
{    
    t = phaseSwg*tSwing;

    trajX = FuncPoly5th(t, 0, tSwing, p0(0), 0, 0, pf(0), 0, 0);
    trajY = FuncPoly5th(t, 0, tSwing, p0(1), 0, 0, pf(1), 0, 0);
    trajZ = FuncPoly6th(t, 0, tSwing, 0, 0, 0, 0, 0, 0, Fh);

    Pf << trajX(0), trajY(0), trajZ(0);
    Vf << trajX(1), trajY(1), trajZ(1);
    Af << trajX(2), trajY(2), trajZ(2);
}

Eigen::Vector3d FootSwingTrajectory::FuncPoly5th(double RealTime, double t_start, double t_end, double Z0, double dZ0, double ddZ0, double Ze, double dZe, double ddZe)
{
    double tw = t_end - t_start;
    double Rt = RealTime - t_start;
    double Pos, Vel, Acc;
    Eigen::Vector3d trajOut;

    double n0 = Z0;
    double n1 = dZ0;
    double n2 = ddZ0 / 2;
    double n3 = -(20 * Z0 - 20 * Ze + 12 * dZ0 * tw + 8 * dZe * tw + 3 * ddZ0 * pow(tw, 2) - ddZe * pow(tw, 2)) / (2 * pow(tw, 3));
    double n4 = (30 * Z0 - 30 * Ze + 16 * dZ0 * tw + 14 * dZe * tw + 3 * ddZ0 * pow(tw, 2) - 2 * ddZe * pow(tw, 2)) / (2 * pow(tw, 4));
    double n5 = -(12 * Z0 - 12 * Ze + 6 * dZ0 * tw + 6 * dZe * tw + ddZ0 * pow(tw, 2) - ddZe * pow(tw, 2)) / (2 * pow(tw, 5));

    if (RealTime >= t_start && RealTime <= t_end)
    {
        Pos = n0 + n1 * Rt + n2 * pow(Rt, 2) + n3 * pow(Rt, 3) + n4 * pow(Rt, 4) + n5 * pow(Rt, 5);
        Vel = n1 + 2 * n2 * Rt + 3 * n3 * pow(Rt, 2) + 4 * n4 * pow(Rt, 3) + 5 * n5 * pow(Rt, 4);
        Acc = 2 * n2 + 6 * n3 * Rt + 12 * n4 * pow(Rt, 2) + 20 * n5 * pow(Rt, 3);
    }
    else if (RealTime < t_start)
    {
        Pos = Z0;
        Vel = dZ0;
        Acc = ddZ0;
    }
    else if (RealTime > t_end)
    {
        Pos = Ze;
        Vel = dZe;
        Acc = ddZe;
    }
    trajOut << Pos, Vel, Acc;
    return trajOut;
}

Eigen::Vector3d FootSwingTrajectory::FuncPoly6th(double RealTime, double t_start, double t_end, double Z0, double dZ0, double ddZ0, double Ze, double dZe, double ddZe, double Fh)
{
    double tw = t_end - t_start;
    double Rt = RealTime - t_start;
    double Pos, Vel, Acc;
    Eigen::Vector3d trajOut;

    double n0 = Z0;
    double n1 = dZ0;
    double n2 = ddZ0 / 2;
    double n3 = -(84 * Z0 - 128 * Fh + 44 * Ze + 32 * dZ0 * tw - 12 * dZe * tw + 5 * ddZ0 * pow(tw, 2) + ddZe * pow(tw, 2)) / (2 * pow(tw, 3));
    double n4 = (222 * Z0 - 384 * Fh + 162 * Ze + 76 * dZ0 * tw - 46 * dZe * tw + 9 * ddZ0 * pow(tw, 2) + 4 * ddZe * pow(tw, 2)) / (2 * pow(tw, 4));
    double n5 = -(204 * Z0 - 384 * Fh + 180 * Ze + 66 * dZ0 * tw - 54 * dZe * tw + 7 * ddZ0 * pow(tw, 2) + 5 * ddZe * pow(tw, 2)) / (2 * pow(tw, 5));
    double n6 = (32 * Z0 - 64 * Fh + 32 * Ze + 10 * dZ0 * tw - 10 * dZe * tw + ddZ0 * pow(tw, 2) + ddZe * pow(tw, 2)) / pow(tw, 6);

    if (RealTime >= t_start && RealTime <= t_end)
    {
        Pos = n0 + n1 * Rt + n2 * pow(Rt, 2) + n3 * pow(Rt, 3) + n4 * pow(Rt, 4) + n5 * pow(Rt, 5) + n6 * pow(Rt, 6);
        Vel = n1 + 2 * n2 * Rt + 3 * n3 * pow(Rt, 2) + 4 * n4 * pow(Rt, 3) + 5 * n5 * pow(Rt, 4) + 6 * n6 * pow(Rt, 5);
        Acc = 2 * n2 + 6 * n3 * Rt + 12 * n4 * pow(Rt, 2) + 20 * n5 * pow(Rt, 3) + 30 * n6 * pow(Rt, 4);
    }
    else if (RealTime < t_start)
    {
        Pos = Z0;
        Vel = dZ0;
        Acc = ddZ0;
    }
    else if (RealTime > t_end)
    {
        Pos = Ze;
        Vel = dZe;
        Acc = ddZe;
    }
    trajOut << Pos, Vel, Acc;
    return trajOut;
}
