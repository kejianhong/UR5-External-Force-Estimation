#include "ExtObserverLib_v3/momentum_observer_rnea.h"

// 构造函数
MomentumObserverRnea::MomentumObserverRnea(RobotDynamicsRnea* rd, Vector& k) : ExternalObserverRnea(rd, ID_MomentumObserverRnea), ko(k), sum(Vector(jointNo)), r(Vector(jointNo)), zero(Vector::Zero(jointNo)), p(Vector(jointNo)), beta(Vector(jointNo)), torque(Vector(jointNo)), t_prev(Vector(jointNo)) {
}

// Torque Estimation
Vector MomentumObserverRnea::getExternalTorque(Vector& q, Vector& qd, Vector& tau, double dt) {
  p = dyn->rnea(q, zero, qd, 0);                                   // M*qd,默认参数Gravity=0
  beta = dyn->rnea(q, zero, zero, GRAVITY) - dyn->tranCqd(q, qd);  // G - C^T*qd
  torque = tau - dyn->getFriction(qd);                             // 先剔除掉摩擦力

  if (isRun) {
    torque += r - beta;                   // r表示上一次外力估计值tau^，整个等式表示积分符号的被积分项，即(tau + C^T*qd - G - F + tau^)
    sum += 0.5 * dt * (torque + t_prev);  // torque表示当前时刻的积分项，t_prev表示上一时刻的积分项，梯形公式求[t-1,t]时刻的积分
  } else {
    // 初始时刻
    r.setZero();
    torque += r - beta;  // 初始时刻，外力估计值是零向量
    sum = p;             // 初始时刻的广义动量，只需要做差一次
    isRun = true;
  }
  t_prev = torque;  // 保存当前时刻的积分项，用于下一次积分
  p -= sum;         // 变量重复利用，p-积分项

  // 权重
  for (int i = 0; i < jointNo; i++) {
    r(i) = ko(i) * p(i);
    torque(i) = r(i);  // 重复利用变量torque，因为每次调用getExternalTorque，该变量都会重新被赋值
  }

  return torque;
}