#include "ExtObserverLib_v3/filtered_dyn_observer_rnea.h"

// 构造函数
FilterDynObserverRnea::FilterDynObserverRnea(RobotDynamicsRnea* rd, double cutOff, double sampTime) : ExternalObserverRnea(rd, ID_FilterDynObserverRnea), f1(FilterF1(cutOff, sampTime, jointNo)), f2(FilterF2(cutOff, sampTime, jointNo)), p(Vector(jointNo)), tauExt(Vector(jointNo)), zeros(Vector::Zero(jointNo)) {
}

Vector FilterDynObserverRnea::getExternalTorque(Vector& q, Vector& qd, Vector& tau, double dt) {
  p = dyn->rnea(q, zeros, qd, 0.0);  // M*qd

  if (isRun) {
    u = dyn->getFriction(qd) + dyn->rnea(q, zeros, zeros, GRAVITY) - dyn->tranCqd(q, qd) - tau;
    tauExt = f1.filt(u, dt) + f2.filt(p, dt) + f2.getOmega() * p;
  } else {
    u = dyn->getFriction(qd) + dyn->rnea(q, zeros, zeros, GRAVITY) - dyn->tranCqd(q, qd) - tau;
    f1.set(u);
    f2.set(p);
    tauExt.setZero();
    isRun = true;
  }
  return tauExt;
}