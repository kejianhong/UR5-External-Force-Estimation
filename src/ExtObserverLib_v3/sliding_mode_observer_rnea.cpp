#include "ExtObserverLib_v3/sliding_mode_observer_rnea.h"

// 初始化构造函数
SlidingModeObserverRnea::SlidingModeObserverRnea(RobotDynamicsRnea* rd, Vector& t1, Vector& s1, Vector& t2, Vector& s2) : ExternalObserverRnea(rd, ID_SlidingModeObserverRnea), sigma(Vector(jointNo)), p_hat(Vector(jointNo)), tauExt(Vector(jointNo)), p(Vector(jointNo)), dispp(Vector(jointNo)), dp_hat(Vector(jointNo)), u(Vector(jointNo)), dsigma(Vector(jointNo)), zeros(Vector::Zero(jointNo)), T1(Vector(jointNo)), S1(Vector(jointNo)), T2(Vector(jointNo)), S2(Vector(jointNo)) {
  T1 = t1;
  T2 = t2;
  S1 = s1;
  S2 = s2;
}

// 力矩估计
Vector SlidingModeObserverRnea::getExternalTorque(Vector& q, Vector& qd, Vector& tau, double dt) {
  p = dyn->rnea(q, zeros, qd, 0);
  u = tau + dyn->tranCqd(q, qd) - dyn->rnea(q, zeros, zeros, GRAVITY) - dyn->getFriction(qd);  // eq26

  if (!isRun) {
    sigma.setZero();
    p_hat = p;  // 初始化p_hat(0)=p(0)
    isRun = true;
  }

  dispp = p_hat - p;  // p_hat - p
  // dp_hat
  for (int i = 0; i < jointNo; i++) {
    dp_hat(i) = u(i) - T1(i) * sqrt(fabs(dispp(i))) * tanh(dispp(i) * BIGR) - T2(i) * dispp(i) + sigma(i);
  }

  // dsigma
  for (int i = 0; i < jointNo; i++) {
    dsigma(i) = -S1(i) * tanh(dispp(i) * BIGR) - S2(i) * dispp(i);
  }

  p_hat += dt * dp_hat;
  sigma += dt * dsigma;
  tauExt = sigma;

  return tauExt;
}