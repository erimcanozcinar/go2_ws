#ifndef MPC_HPP
#define MPC_HPP

#include "eigen3/Eigen/Dense"
#include "eigen3/unsupported/Eigen/MatrixFunctions"
#include <qpOASES.hpp>
#include "legged_common/DataTypes.hpp"
#include "legged_trajectory/GaitScheduler.hpp"

#define BIG_NUM 5e10
#define GRAVITY 9.81
#define MASS 15.098

class ModelPredictiveControl {
    private:
    qpOASES::SQProblem* mpcProblem;
	qpOASES::Options op;  
    
    std::array<Eigen::Vector3d, 4> footForce;
    std::array<Eigen::Vector3d, 4> pCom2Foot;

    Eigen::Vector3d gravityDirectionVector;
    int horizon = 10;
    int iterCount = 0;
    double dt;
    int iterBetweenMPC;
    double dtMPC;

    Eigen::Matrix3d I_body, I_world;
    double mu = 0.6;

    int n = 0, m = 0;

    bool isFirstIter = true;

    public:
    EstimatorData* _est;

    Eigen::MatrixXd Ac, Bc, ABc, expMat;    
    Eigen::MatrixXd Ad, Bd;    
    Eigen::MatrixXd Aqp, Bqp;
    Eigen::MatrixXd H, L, K, C;
    Eigen::MatrixXd fmat;
    Eigen::VectorXd diagL;
    Eigen::VectorXd xref, x0, g, y, ubC, ulC;
    Eigen::Vector3d pCOM_des, vCOM_des, omega_des;
    Eigen::Vector3d rpy_des, rpy_int, rpy_comp;

    Eigen::VectorXd optF;

    qpOASES::real_t H_qp[120*120], g_qp[120*1];
	qpOASES::real_t C_qp[200*120];
	qpOASES::real_t ulC_qp[200*1], ubC_qp[200*1];
	qpOASES::real_t fOpt[120*1];
	qpOASES::int_t nWSR = 100;

    std::array<Eigen::Vector4d, 10> gait;
    std::array<Eigen::Vector4i, 250> gaitTable;
    int global_gait_index = 0; // 0 ile 249 arasında değişir

    ModelPredictiveControl(EstimatorData* _estData = nullptr, double _dt = 0.001, int _iterBetweenMPC = 25);
    void run(const Gait* _gait);
    const std::array<Eigen::Vector3d, 4>& getFootForces() const { return footForce; }
    void discreteModel(double yaw);
    void qpForm(double yaw);
    void matrix_to_real(qpOASES::real_t* outM, Eigen::MatrixXd inM);
    void solve_mpc(double yaw);
    void setDesiredStates(DesiredStates* _desStates);
    void setGaitTable(const Gait* ptrGait);
    void updateData();
    void compensationRollPitch();

};

#endif