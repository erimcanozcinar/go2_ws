#include "legged_common/matTools.hpp"

// Calculates world-to-body rotation matrix from quaternion.
Eigen::Matrix3d quaternionToRotationMatrix( const Eigen::Vector4d& q) {

  double e0 = q(0);
  double e1 = q(1);
  double e2 = q(2);
  double e3 = q(3);

  Eigen::Matrix3d R;

  R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
      2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
      1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
      2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
      1 - 2 * (e1 * e1 + e2 * e2);
  R.transposeInPlace(); // Transpose to get world-to-body rotation
  return R;
}

Eigen::Vector3d quaternionToso3(const Eigen::Vector4d& quat) {
  Eigen::Vector3d so3 = Eigen::Vector3d::Zero();
  so3(0) = quat(1);
  so3(1) = quat(2);
  so3(2) = quat(3);

  double theta =
      2.0 * asin(sqrt(so3(0) * so3(0) + so3(1) * so3(1) + so3(2) * so3(2)));

  if (fabs(theta) < 0.0000001) {
    so3.setZero();
    return so3;
  }
  so3 /= sin(theta / 2.0);
  so3 *= theta;
  return so3;
}

// Quaternion to Euler angles (calculates as ZYX but retuns XYZ)
Eigen::Vector3d quatToRPY(const Eigen::Vector4d& q) {
  Eigen::Vector3d rpy;
  double as = std::min(-2. * (q[1] * q[3] - q[0] * q[2]), .99999);
  rpy(2) = std::atan2(2 * (q[1] * q[2] + q[0] * q[3]),
                  pow(q[0],2) + pow(q[1],2) - pow(q[2],2) - pow(q[3],2));
  rpy(1) = std::asin(as);
  rpy(0) =
      std::atan2(2 * (q[2] * q[3] + q[0] * q[1]),
                  pow(q[0],2) - pow(q[1],2) - pow(q[2],2) + pow(q[3],2));
  return rpy;
}

// Input is rotation matrix from world to body
Eigen::Vector4d rotationMatrixToQuaternion(const Eigen::Matrix3d& r1) {
  Eigen::Vector4d q;
  Eigen::Matrix3d r = r1.transpose();
  double tr = r.trace();
  if (tr > 0.0) {
    double S = sqrt(tr + 1.0) * 2.0;
    q(0) = 0.25 * S;
    q(1) = (r(2, 1) - r(1, 2)) / S;
    q(2) = (r(0, 2) - r(2, 0)) / S;
    q(3) = (r(1, 0) - r(0, 1)) / S;
  } else if ((r(0, 0) > r(1, 1)) && (r(0, 0) > r(2, 2))) {
    double S = sqrt(1.0 + r(0, 0) - r(1, 1) - r(2, 2)) * 2.0;
    q(0) = (r(2, 1) - r(1, 2)) / S;
    q(1) = 0.25 * S;
    q(2) = (r(0, 1) + r(1, 0)) / S;
    q(3) = (r(0, 2) + r(2, 0)) / S;
  } else if (r(1, 1) > r(2, 2)) {
    double S = sqrt(1.0 + r(1, 1) - r(0, 0) - r(2, 2)) * 2.0;
    q(0) = (r(0, 2) - r(2, 0)) / S;
    q(1) = (r(0, 1) + r(1, 0)) / S;
    q(2) = 0.25 * S;
    q(3) = (r(1, 2) + r(2, 1)) / S;
  } else {
    double S = sqrt(1.0 + r(2, 2) - r(0, 0) - r(1, 1)) * 2.0;
    q(0) = (r(1, 0) - r(0, 1)) / S;
    q(1) = (r(0, 2) + r(2, 0)) / S;
    q(2) = (r(1, 2) + r(2, 1)) / S;
    q(3) = 0.25 * S;
  }
  return q;
}

Eigen::Vector4d rpyToQuat(const Eigen::Vector3d& rpy) {
    Eigen::Matrix3d R = (RotateYaw(rpy(2))*RotatePitch(rpy(1))*RotateRoll(rpy(0))).transpose();
    Eigen::Vector4d q = rotationMatrixToQuaternion(R);
    return q;
}

Eigen::Vector4d quatProduct(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2) {
    double r1 = q1[0];
    double r2 = q2[0];
    Eigen::Vector3d v1(q1[1], q1[2], q1[3]);
    Eigen::Vector3d v2(q2[1], q2[2], q2[3]);

    double r = r1 * r2 - v1.dot(v2);
    Eigen::Vector3d v = r1 * v2 + r2 * v1 + v1.cross(v2);
    Eigen::Vector4d q(r, v(0), v(1), v(2));
    return q;
}

// Rotate roll body to world frame
Eigen::Matrix3d RotateRoll(double a) {
    double RotX[3][3];
    Eigen::Matrix3d rollMatrix;

    RotX[0][0] = 1;
    RotX[0][1] = 0;
    RotX[0][2] = 0;

    RotX[1][0] = 0;
    RotX[1][1] = cos(a);
    RotX[1][2] = -sin(a);

    RotX[2][0] = 0;
    RotX[2][1] = sin(a);
    RotX[2][2] = cos(a);

    rollMatrix << RotX[0][0], RotX[0][1], RotX[0][2], RotX[1][0], RotX[1][1], RotX[1][2], RotX[2][0], RotX[2][1], RotX[2][2];
    return rollMatrix;
}

// Rotate pitch body to world frame
Eigen::Matrix3d RotatePitch(double a) {
    double RotY[3][3];
    Eigen::Matrix3d pitchMatrix;

    RotY[0][0] = cos(a);
    RotY[0][1] = 0;
    RotY[0][2] = sin(a);

    RotY[1][0] = 0;
    RotY[1][1] = 1;
    RotY[1][2] = 0;

    RotY[2][0] = -sin(a);
    RotY[2][1] = 0;
    RotY[2][2] = cos(a);

    pitchMatrix << RotY[0][0], RotY[0][1], RotY[0][2], RotY[1][0], RotY[1][1], RotY[1][2], RotY[2][0], RotY[2][1], RotY[2][2];
    return pitchMatrix;
}

// Rotate yaw body to world frame
Eigen::Matrix3d RotateYaw(double a) {
    double RotZ[3][3];
    Eigen::Matrix3d yawMatrix;

    RotZ[0][0] = cos(a);
    RotZ[0][1] = -sin(a);
    RotZ[0][2] = 0;

    RotZ[1][0] = sin(a);
    RotZ[1][1] = cos(a);
    RotZ[1][2] = 0;

    RotZ[2][0] = 0;
    RotZ[2][1] = 0;
    RotZ[2][2] = 1;

    yawMatrix << RotZ[0][0], RotZ[0][1], RotZ[0][2], RotZ[1][0], RotZ[1][1], RotZ[1][2], RotZ[2][0], RotZ[2][1], RotZ[2][2];
    return yawMatrix;
}

Eigen::Matrix3d vec2SkewSym(Eigen::Vector3d V)
{
    Eigen::Matrix3d M;
    M << 0, -V(2), V(1),
         V(2), 0, -V(0),
         -V(1), V(0), 0;
    return M;
}

double funUnwrap(double angle, double prevAngle, int &cumSumdeltaCorr) {
    int k = 0;
    double deltaCorr = 0;
    double Out;

    deltaCorr = (angle - prevAngle)/(2*M_PI);
    k = round(deltaCorr);
    cumSumdeltaCorr = cumSumdeltaCorr + k;

    Out = angle - 2*M_PI*cumSumdeltaCorr;
    return Out;
}

double numIntegral(double In, double prevIn, double prevOut, double dt)
{
    double Out = prevOut + (In + prevIn) * dt / 2;
    return Out;
}

double Numdiff(double currX, double prevX, double dt)
{
    return (currX - prevX) / dt;
}

double LPF(double Input, double prevOut, double freq, double dt) {
    double out = (dt * freq * Input + prevOut) / (1 + freq * dt);
    return out;
}

void pseudoInverse(const Eigen::MatrixXd& A, double threshold, Eigen::MatrixXd& Ainv) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  int n = svd.singularValues().size();
  Eigen::VectorXd invS = Eigen::VectorXd::Zero(n);
  
  for (int i = 0; i < n; ++i) {
      if (svd.singularValues()(i) > threshold) {
          invS(i) = 1.0 / svd.singularValues()(i);
      }
  }
  Ainv = svd.matrixV() * invS.asDiagonal() * svd.matrixU().transpose();
}

void weightedPseudoInverse(const Eigen::MatrixXd& A, const Eigen::MatrixXd& Winv, Eigen::MatrixXd& Ainv, double threshold) {
  Eigen::MatrixXd lambda(A*Winv*A.transpose());
  Eigen::MatrixXd lambda_inv;
  pseudoInverse(lambda, threshold, lambda_inv);
  Ainv = Winv*A.transpose()*lambda_inv;
}

