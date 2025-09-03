#include "ExtObserverLib_v3/nonlinear_disturbance_observer_rnea.h"

// 构造函数初始化
DisturbanceObserverRnea::DisturbanceObserverRnea(RobotDynamicsRnea* rd, double sigma, double xeta, double beta) : ExternalObserverRnea(rd, ID_DisturbanceObserverRnea), Y(Matrix(jointNo, jointNo)), L(Matrix(jointNo, jointNo)), I(Matrix::Identity(jointNo, jointNo)), lft(Matrix(jointNo, jointNo)), rht(Matrix(jointNo, jointNo)), phi(Vector(jointNo)), z(Vector(jointNo)), torque(Vector(jointNo)), torqueEst(Vector(jointNo)), zeros(Vector::Zero(jointNo)) {
  Y = 0.5 * (xeta + 2 * beta * sigma) * Matrix::Identity(jointNo, jointNo);  // eq21，定植
}

// Torque Estimation
Vector DisturbanceObserverRnea::getExternalTorque(Vector& q, Vector& qd, Vector& tau, double dt) {
  L = Y * dyn->getM(q).inverse();  // eq19: L = inv(X)*inv(M) = Y * inv(M)
  L *= dt;                         // 针对eq18的处理，公式两边同时乘以dt
  phi = Y * qd;                    // eq19: phi = inv(X)*qd = Y*qd

  // eq18，两边同时乘以时间步长，整理得到
  // z(i) = inv(I+L*dt) * (z(i-1)+L*dt*(C*qd+G+F-tau-phi))
  // 记为 z(i) = lft + rht
  // 力估计等式为：torqueEst = z + phi，初始估计值为０
  if (isRun) {
    lft = I + L;
    rht = z + L * (dyn->rnea(q, qd, zeros, GRAVITY) + dyn->getFriction(qd) - tau - phi);
    // 更新z
    z = lft.inverse() * rht;
  } else {
    // 初始时刻外力估计值为0，所以z(0)+phi=0,z(0)=-phi
    z = -phi;
    isRun = true;
  }
  torqueEst = z + phi;
  return torqueEst;
}