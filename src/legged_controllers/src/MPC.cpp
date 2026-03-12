#include "legged_controllers/MPC.hpp"
#include "legged_common/matTools.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

ModelPredictiveControl::ModelPredictiveControl(EstimatorData* _estData, double _dt, int _iterBetweenMPC) : dt(_dt), iterBetweenMPC(_iterBetweenMPC), _est(_estData){
    // MPC time step: time between MPC solves (typically 0.05s for 20Hz MPC rate)
    dtMPC = dt*iterBetweenMPC;
    
    // Initialize qpOASES QP solver
    // Problem size: 120 variables (12 forces * 10 horizon steps), 200 constraints (5 constraints * 4 feet * 10 steps)
    mpcProblem = new qpOASES::SQProblem(120,200);
    op.setToMPC();  // Set solver options optimized for MPC problems
	op.printLevel = qpOASES::PL_NONE;  // Suppress solver output
	mpcProblem->setOptions(op);

    // Initialize optimal force vector (12 components: 3 forces per foot * 4 feet)
    optF.resize(12);
    optF.setZero();

    // Continuous-time state-space matrices
    // Ac: 13x13 state transition matrix (13 states: rpy, pos, omega, v, gravity)
    // Bc: 13x12 input matrix (12 inputs: 3 forces per foot * 4 feet)
    Ac.resize(13,13); 
    Bc.resize(13,12); 
    n = Ac.rows(); // Number of states (13)
    m = Bc.cols(); // Number of inputs (12)

    // Discrete-time matrices and matrix exponential workspace
    ABc.resize(n+m,n+m);  // Augmented matrix for matrix exponential
    expMat.resize(n+m,n+m);  // Result of matrix exponential
    ABc.setZero(); 
    expMat.setZero();
    Ad.resize(n,n);  // Discrete state transition matrix
    Bd.resize(n,m);  // Discrete input matrix
    
    // Prediction matrices for QP formulation
    // Aqp: maps initial state to predicted states over horizon
    // Bqp: maps input sequence to predicted states over horizon
    Aqp.resize(Ad.rows()*horizon, Ad.cols());
    Bqp.resize(Ad.rows()*horizon, Bd.cols()*horizon);

    // Constraint matrices
    fmat.resize(5,3);  // Friction cone constraint matrix (5 constraints per foot)
    ubC.resize(fmat.rows()*4*horizon);  // Upper bound vector for constraints
    ulC.resize(fmat.rows()*4*horizon);  // Lower bound vector for constraints
    ulC.setZero(); 
    ubC.setZero();

    // QP problem matrices
    H.resize(m*horizon,m*horizon);  // Hessian matrix (120x120)
    g.resize(m*horizon);  // Gradient vector (120x1) - FIXED: was 3*4*horizon, should be m*horizon
    y.resize(n*horizon);  // Reference trajectory over horizon
    x0.resize(n);  // Initial state
    xref.resize(n);  // Reference state
    L.resize(n*horizon,n*horizon);  // State tracking weight matrix
    K.resize(m*horizon,m*horizon);  // Force regularization weight matrix
    diagL.resize(n);  // Diagonal state weights
    C.resize(fmat.rows()*horizon*4,fmat.cols()*horizon*4);  // Constraint matrix

    // Initialize all matrices to zero
    Ac.setZero(); 
    Bc.setZero();
    Ad.setZero(); 
    Bd.setZero();
    Aqp.setZero(); 
    Bqp.setZero();
    H.setZero(); 
    g.setZero(); 
    y.setZero(); 
    x0.setZero(); 
    xref.setZero();
    
    // Initialize default state values
    x0(5) = 0.3;        // Initial z position (height)
    x0(12) = -GRAVITY;  // Gravity state
    xref(5) = 0.3;      // Reference z position
    xref(12) = -GRAVITY; // Reference gravity
    
    // Cost function weights
    // K: Force regularization weight (penalizes large forces)
    // Small value (1e-5) allows forces to be used when needed but prevents excessive forces
    K = 1e-5*Eigen::MatrixXd::Identity(m*horizon,m*horizon);
    
    // L: State tracking weights (diagonal matrix)
    // Weights for: [rpy(3), pos(3), omega(3), v(3), gravity(1)]
    // Higher weights mean tighter tracking of that state
    diagL << 0.25,   // roll weight
            0.25,   // pitch weight
            10,     // yaw weight (higher - important for heading)
            2,      // x position weight
            2,      // y position weight
            50,     // z position weight (much higher - critical for height control)
            0,      // omega_x weight (not directly penalized)
            0,      // omega_y weight (not directly penalized)
            0.3,    // omega_z weight
            0.2,    // v_x weight
            0.2,    // v_y weight
            0.1,    // v_z weight
            0;      // gravity weight (not penalized, it's constant)
    
    // Replicate weights for entire horizon
    L.diagonal() = diagL.replicate(horizon,1);
    
    // Initialize reference trajectory
    for(int i=0; i<horizon; i++){
        y.block(i*n,0,n,1) = xref;
    }    

    I_body << 0.02448, 0, 0,
              0, 0.098077, 0,
              0, 0, 0.107;


    rpy_int.setZero();
    rpy_comp.setZero();

    gravityDirectionVector << 0, 0, 1;	
    
}

void ModelPredictiveControl::discreteModel(double yaw){
    // Rotate inertia tensor to world frame (only yaw rotation for simplified model)
    I_world = RotateYaw(yaw)*I_body*RotateYaw(yaw).transpose();

    // Continuous-time state-space model: x_dot = Ac*x + Bc*u
    // State vector: [rpy(3), pos(3), omega(3), v(3), gravity(1)]
    Ac.setZero();
    
    // Position dynamics: pos_dot = v
    Ac(3,9) = 1.0;  // pos_x_dot = v_x
    Ac(4,10) = 1.0; // pos_y_dot = v_y
    Ac(5,11) = 1.0; // pos_z_dot = v_z
    
    // Orientation dynamics: rpy_dot = R_yaw^T * omega (linearized for small angles)
    // This is the simplified model from the paper
    Ac.block(0,6,3,3) = RotateYaw(yaw).transpose();
    
    // Linear velocity dynamics: v_z_dot includes gravity term
    // Gravity is treated as a state (x[12] = -GRAVITY) for the linearized model
    Ac(11,12) = 1.0; // v_z_dot = gravity (gravity state is -9.81, so this gives -9.81 acceleration)
    
    // Input matrix Bc: maps ground reaction forces to state derivatives
    Bc.setZero();
    Eigen::Matrix3d I_inv = I_world.inverse();

    // For each foot, compute the effect of ground reaction forces
    for(int i = 0; i < 4; i++) {
        // Angular acceleration: omega_dot = I^-1 * sum(r_i x f_i)
        // where r_i is the vector from COM to foot i
        Bc.block(6,i*3,3,3) = I_inv*vec2SkewSym(_est->rBody2World*_est->pFootBody[i]);
        
        // Linear acceleration: v_dot = (1/m) * sum(f_i)
        Bc.block(9,i*3,3,3) = Eigen::Matrix3d::Identity() / MASS;
    }
    
    // Discrete State-Space by using Zero-Order Hold (ZOH)
    // This matches the paper's discretization approach
    ABc.setZero();
    ABc.block(0,0,n,n) = Ac;
    ABc.block(0,n,n,m) = Bc;
    ABc = dtMPC*ABc;  // Scale by MPC time step
    
    // Matrix exponential: exp(A*dt) gives the discrete-time transition matrix
    expMat = ABc.exp();
    Ad = expMat.block(0,0,n,n);  // Discrete state transition matrix
    Bd = expMat.block(0,n,n,m);  // Discrete input matrix
}

void ModelPredictiveControl::qpForm(double yaw){
    // Update discrete-time model matrices
    discreteModel(yaw);
    
    // Precompute powers of Ad for efficient prediction matrix construction
    // powerMats[i] = Ad^i
    Eigen::Matrix<double,13,13> powerMats[20];
    powerMats[0].setIdentity();
    for(int i = 1; i < horizon+1; i++){
        powerMats[i] = Ad*powerMats[i-1];
    }

    // Build prediction matrices Aqp and Bqp
    // These matrices relate the initial state and input sequence to the predicted states
    // x_predicted = Aqp*x0 + Bqp*u_sequence
    for (int r = 0; r < horizon; ++r) {
        // State prediction at step r+1: x[r+1] = Ad^(r+1) * x0
        Aqp.block(n*r,0,n,n) = powerMats[r+1];
        
        // Input-to-state mapping: contribution of inputs at each step
        for(int c = 0; c < horizon; c++){
            if(r >= c){
                // Input at step c affects state at step r+1 via Ad^(r-c)*Bd
                Bqp.block(n*r,m*c,n,m) = powerMats[r-c]*Bd;
            }
        }
    }

    // QP cost function: minimize (x - x_ref)^T * L * (x - x_ref) + u^T * K * u
    // Substituting x = Aqp*x0 + Bqp*u, we get:
    // cost = (Bqp*u + Aqp*x0 - y)^T * L * (Bqp*u + Aqp*x0 - y) + u^T * K * u
    // Expanding: cost = u^T*(Bqp^T*L*Bqp + K)*u + 2*(Aqp*x0 - y)^T*L*Bqp*u + constant
    // Standard QP form: (1/2)*u^T*H*u + g^T*u
    H = 2*(Bqp.transpose()*L*Bqp + K);  // Hessian matrix
    g = 2*Bqp.transpose()*L*(Aqp*x0 - y); // Gradient vector
}

void ModelPredictiveControl::matrix_to_real(qpOASES::real_t* outM, Eigen::MatrixXd inM){
    // Convert Eigen matrix to qpOASES real_t array (row-major format)
    // qpOASES expects matrices in row-major order (C-style)
    int rows = inM.rows();
    int cols = inM.cols();
    int c = 0;

    for(int i=0; i<rows; i++){
        for(int j=0; j<cols; j++){
            outM[c] = inM(i,j);
            c++;
        }
    }
}

void ModelPredictiveControl::solve_mpc(double yaw){
    // Set up constraint bounds for friction cone and normal force constraints
    // For each foot at each horizon step, we have 5 constraints:
    // 4 linearized friction cone constraints + 1 normal force constraint
    int k = 0;
    for(int i=0; i<horizon; i++){
        for(int j=0; j<4; j++){
            // Friction cone constraints: multiply with gait (always BIG_NUM to keep constraints disabled)
            ubC(5*k+0) = BIG_NUM;  // Always equals BIG_NUM
            ubC(5*k+1) = BIG_NUM;
            ubC(5*k+2) = BIG_NUM;
            ubC(5*k+3) = BIG_NUM;
            
            // Normal force constraint: multiply with gait to enable/disable based on contact
            // When gait[i](j) = 0 (not in contact): ubC = 0, ulC = 0 -> forces f_z = 0
            // When gait[i](j) = 1 (in contact): ubC = 120, ulC = 0 -> allows 0 <= f_z <= 120
            ubC(5*k+4) = gait[i](j)*120;
            ulC(5*k+4) = gait[i](j)*0;

            // Lower bounds for friction constraints: multiply with gait (always -BIG_NUM)
            ulC(5*k+0) = -BIG_NUM;  // Always equals -BIG_NUM
            ulC(5*k+1) = -BIG_NUM;
            ulC(5*k+2) = -BIG_NUM;
            ulC(5*k+3) = -BIG_NUM;
            k++;
        }
    }

    // Friction cone constraint matrix (linearized pyramid approximation)
    // Each row represents one linearized constraint: [f_x_coeff, f_y_coeff, f_z_coeff]
    // Original formulation: constraints are effectively disabled with BIG_NUM bounds
    // Note: This formulation may not actively enforce friction cone, relying on other mechanisms
    fmat << 1/mu,  0,  1,   // Original formulation
           -1/mu,  0,  1,   
            0,  1/mu,  1,   
            0, -1/mu,  1,   
            0,     0,  1;   // f_z constraint (normal force)
    
    // Build constraint matrix C for all feet at all horizon steps
    // C*u <= ubC, C*u >= ulC
    for (int i=0; i < horizon*4; i++){
        C.block(i*fmat.rows(),i*fmat.cols(), fmat.rows(), fmat.cols()) = fmat;
    }

    // Formulate QP problem
    qpForm(yaw);
    
    // Convert Eigen matrices to qpOASES format (row-major)
    matrix_to_real(H_qp,H);
	matrix_to_real(g_qp,g);
	matrix_to_real(C_qp,C);
	matrix_to_real(ulC_qp,ulC);
	matrix_to_real(ubC_qp,ubC);
    
    // Solve QP using qpOASES
    nWSR = 100;  // Maximum number of working set recalculations
    qpOASES::returnValue retVal;
    if(isFirstIter) {
        // First iteration: initialize solver
        retVal = mpcProblem->init(H_qp, g_qp, C_qp, NULL, NULL, ulC_qp, ubC_qp, nWSR);
        isFirstIter = false;
    } else {
        // Subsequent iterations: use hot-start for faster convergence
        retVal = mpcProblem->hotstart(H_qp, g_qp, C_qp, NULL, NULL, ulC_qp, ubC_qp, nWSR);
    }
    if(retVal != qpOASES::SUCCESSFUL_RETURN){
        printf("\033[1;31mfailed to solve mpc!\033[0m\n");
    }
    
    // Extract optimal solution
    mpcProblem->getPrimalSolution(fOpt);

    // Store optimal forces for all 4 feet (12 total force components)
    for (int i = 0; i < 12; i++) {
        optF(i) = fOpt[i];
    }
}

void ModelPredictiveControl::setDesiredStates(DesiredStates* _desStates){
    pCOM_des = _desStates->pos_des;
    vCOM_des = _desStates->vWorld_des;
    omega_des = _desStates->omegaWorld_des;
    rpy_des = _desStates->rpy_des;
}

void ModelPredictiveControl::setGaitTable(const Gait* ptrGait){
    // Gait objesinden genel periyodu al
    double gaitPeriod = ptrGait->getGaitPeriod();
        
    for (int k = 0; k < horizon; ++k) {
        // Gelecekteki zaman adımı (prediction)
        double predictionTime = k * dtMPC;
        double phase_advance = predictionTime / gaitPeriod;

        for(int leg = 0; leg < 4; ++leg) {
            // O anki bacak fazını ve switching phase'i direkt objeden çek
            double currentPhase = ptrGait->getPhase(leg);
            double switchingPhase = ptrGait->getSwitchingPhase();

            // Gelecekteki fazı hesapla
            double future_phase = std::fmod(currentPhase + phase_advance, 1.0);
            if(future_phase < 0) future_phase += 1.0;

            // Contact durumunu belirle (1: Yerde/Stance, 0: Havada/Swing)
            // Not: Sınıf üyesi olan 'gait' dizisi ile parametreyi karıştırmayın
            if(future_phase <= switchingPhase) {
                gait[k](leg) = 1; 
            } else {
                gait[k](leg) = 0;
            }
        }
    }
}

void ModelPredictiveControl::updateData(){
    
    // Set initial state vector: [rpy, pos, omega, v, gravity]
    x0.block(0,0,3,1) = _est->rpy;           // Current orientation
    x0.block(3,0,3,1) = _est->pos;          // Current position
    x0.block(6,0,3,1) = _est->omegaWorld;   // Current angular velocity
    x0.block(9,0,3,1) = _est->vWorld;       // Current linear velocity
    x0(12) = -GRAVITY;                      // Gravity (constant state)

    

    // Set reference state (desired state at current time)
    xref.block(0,0,3,1) = rpy_comp;        // Compensated orientation reference
    xref.block(3,0,3,1) = pCOM_des;        // Desired position
    xref.block(6,0,3,1) = omega_des;       // Desired angular velocity
    xref.block(9,0,3,1) = vCOM_des;        // Desired linear velocity
    xref(12) = -GRAVITY;                    // Gravity (constant)
    
    // Build reference trajectory for entire prediction horizon
    // This creates a sequence of desired states over the horizon
    for(int i=0; i<horizon; i++){
        // Start with base reference
        y.block(i*n,0,n,1) = xref;
        
        if(i == 0) {
            // First step: use current yaw (don't integrate yet)
            y(2) = _est->rpy(2);
        } else {
            // Integrate yaw, x, and y positions forward based on desired velocities
            // This creates a smooth reference trajectory
            y(n*i + 2) = y(n*(i-1) + 2) + dtMPC*omega_des(2);  // Yaw integration
            y(n*i + 3) = y(n*(i-1) + 3) + dtMPC*vCOM_des(0);   // X position integration
            y(n*i + 4) = y(n*(i-1) + 4) + dtMPC*vCOM_des(1);   // Y position integration
        }
    } 
}

void ModelPredictiveControl::compensationRollPitch() {
    if(std::fabs(_est->vWorld(1)) > 0.1) rpy_int(0) += dt*(rpy_des(0) - _est->rpy(0))/_est->vWorld(1);
    if(std::fabs(_est->vWorld(0)) > 0.2) rpy_int(1) += dt*(rpy_des(1) - _est->rpy(1))/_est->vWorld(0);
    rpy_int(0) = fminf(fmaxf(rpy_int(0), -.25), .25);
    rpy_int(1) = fminf(fmaxf(rpy_int(1), -.25), .25);

    rpy_comp(0) = _est->vWorld(1)*rpy_int(0);
    rpy_comp(1) = _est->vWorld(0)*rpy_int(1);
    rpy_comp(2) = rpy_des(2);
}

void ModelPredictiveControl::run(const Gait* _gait){
    // Apply roll/pitch compensation for better tracking
    compensationRollPitch();
    // Solve MPC at the specified rate (typically every iterBetweenMPC control steps)
    if((iterCount % iterBetweenMPC) == 0){
        setGaitTable(_gait);
        updateData();
        solve_mpc(_est->rpy(2));  // Pass current yaw for dynamics linearization
    }
    
    // Transform optimal forces from world frame to body frame
    // Forces are computed in world frame by MPC, but need to be in body frame for control
    // The negative sign accounts for the fact that forces act on the ground, not the robot
    for (int i = 0; i < 4; i++) {
        footForce[i] = -_est->rWorld2Body*optF.segment(3 * i, 3);
    }
    iterCount++;
}