#ifndef MATTOOLS_HPP
#define MATTOOLS_HPP

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

bool areDoubleSame(double a, double b);

Eigen::Matrix3d quaternionToRotationMatrix( const Eigen::Vector4d& q);
Eigen::Vector3d quaternionToso3(const Eigen::Vector4d& quat);
Eigen::Vector3d quatToRPY(const Eigen::Vector4d& q);
Eigen::Vector4d rotationMatrixToQuaternion(const Eigen::Matrix3d& r1);
Eigen::Vector4d rpyToQuat(const Eigen::Vector3d& rpy);
Eigen::Vector4d quatProduct(const Eigen::Vector4d& q1, const Eigen::Vector4d& q2);
Eigen::Matrix3d RotateRoll(double a);
Eigen::Matrix3d RotatePitch(double a);
Eigen::Matrix3d RotateYaw(double a);
Eigen::Matrix3d vec2SkewSym(Eigen::Vector3d V);
double funUnwrap(double angle, double prevAngle, int &cumSumdeltaCorr);
double numIntegral(double In, double prevIn, double prevOut, double dt);
double Numdiff(double currX, double prevX, double dt);
double LPF(double Input, double prevOut, double freq, double dt);
void pseudoInverse(const Eigen::MatrixXd& A, double threshold, Eigen::MatrixXd& Ainv);
void weightedPseudoInverse(const Eigen::MatrixXd& A, const Eigen::MatrixXd& Winv, Eigen::MatrixXd& Ainv, double threshold = 0.0001);

#endif