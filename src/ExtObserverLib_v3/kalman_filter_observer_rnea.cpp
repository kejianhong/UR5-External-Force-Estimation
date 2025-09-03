#include "ExtObserverLib_v3/kalman_filter_observer_rnea.h"

/**
 * @brief 构造函数
 * @param rd 机器人实例指针
 * @param j 雅可比矩阵的转置(可取单位矩阵)
 * @param s 扰动矩阵(一般取0矩阵)
 * @param q 过程噪声协方差矩阵：Q[Qc(摩擦力相关的协方差) 0; 0 Qf(外力扰动的协方差)]
 * @param r 观测噪声的协方差矩阵：测量广义动量的速度噪声协方差
 */
KalmanObserverRnea::KalmanObserverRnea(RobotDynamicsRnea* rd, Matrix& j, Matrix& s, Matrix& q, Matrix& r)
    : ExternalObserverRnea(rd, ID_KalmanObserverRnea),
      J(j),
      X(Vector(2 * jointNo)),  // 对于六自由度机器人来说，p和f(外部六维力矩)维度一样
      u(Vector(jointNo)),      // 控制输入，和机器人自由度维数一样
      p(Vector(jointNo)),
      zero(Vector::Zero(jointNo)) {
  // A[0, J; 0, s] B[I; 0]; C[I; 0]
  Matrix A(2 * jointNo, 2 * jointNo), B(2 * jointNo, jointNo), C(jointNo, 2 * jointNo);
  A.setZero();
  B.setZero();
  C.setZero();
  for (int i = 0; i < jointNo; i++) {
    B(i, i) = 1;
    C(i, i) = 1;
  }
  A.block(0, jointNo, jointNo, jointNo) = j;
  A.block(jointNo, jointNo, jointNo, jointNo) = s;

  // 初始化滤波器
  filter = std::make_shared<KalmanFilter>(KalmanFilter(A, B, C, q, r));
}

Vector KalmanObserverRnea::getExternalTorque(Vector& q, Vector& qd, Vector& tau, double dt) {
  // 计算广义动量
  p = dyn->rnea(q, zero, qd);
  u = tau - dyn->rnea(q, zero, zero, GRAVITY) - dyn->getFriction(qd) + dyn->tranCqd(q, qd);  // 输入控制变量

  if (isRun) {
    X = filter->step(u, p, dt);
  } else {
    X.setZero();
    for (int i = 0; i < jointNo; i++) {
      X(i) = p(i);
    }
    filter->reset(X);
    isRun = true;
  }

  // 估计外力
  p = J * X.block(jointNo, 0, jointNo, 1);  // 重复利用变量p，tau_ext = Jf
  return p;
}
