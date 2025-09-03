#include "ExtObserverLib_v3/kalman_filter_zero_order.h"

Eigen::MatrixXd KalmanFilterZeroOrder::exponential(Eigen::MatrixXd& m, double dt) {
  // exp(m*dt) = 累加((m*dt)^n/n!)
  Eigen::MatrixXd res = Eigen::MatrixXd::Identity(m.rows(), m.cols());
  Eigen::MatrixXd zhankaixiang = res;

  // 展开项数5
  for (int i = 1; i < 5; i++) {
    zhankaixiang *= (m * dt) / i;
    res += zhankaixiang;
  }

  return res;
}

void KalmanFilterZeroOrder::makeDiscrete(double dt) {
  ABd = exponential(AB, dt);
  Ad = ABd.block(0, 0, na, na);
  Bd = ABd.block(0, na, na, nb);
  Rd = R / dt;

  AQd = exponential(AQ, dt);
  Qd = AQd.block(0, na, na, na) * AQd.block(0, 0, na, na).transpose();  // M12 * M11'
}

Eigen::VectorXd KalmanFilterZeroOrder::step(Eigen::VectorXd& u, Eigen::VectorXd& y, double dt) {
  // 离散化
  makeDiscrete(dt);
  // 预测
  X = Ad * X + Bd * u;
  P = Ad * P * Ad.transpose() + Qd;
  // 更新
  Y = Cd * P * Cd.transpose() + Rd;
  K = P * Cd.transpose() * Y.inverse();  // 卡尔曼增益
  // 改正
  X += K * (y - Cd * X);
  P = (I - K * Cd) * P;

  return X;
}