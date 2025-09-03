#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <eigen3/Eigen/Geometry>

/**
 * @brief Discrete Kalman filter implementation
 */
class KalmanFilter {
 public:
  /**
   * @brief 构造函数
   * @param a 状态矩阵
   * @param b 控制输入矩阵
   * @param c 观测矩阵
   */
  KalmanFilter(Eigen::MatrixXd& a, Eigen::MatrixXd& b, Eigen::MatrixXd& c, Eigen::MatrixXd& q, Eigen::MatrixXd& r)
      : A(a),
        B(b),
        C(c),
        // Q(Eigen::MatrixXd(1,1)), R(Eigen::MatrixXd(1,1)),
        Q(q),
        R(r),
        P(Eigen::MatrixXd(1, 1)),
        K(Eigen::MatrixXd(1, 1)),
        Y(Eigen::MatrixXd(1, 1)),
        I(Eigen::MatrixXd(1, 1)),
        Phi(a),
        G(b),
        X(Eigen::VectorXd(1)) {
    nx = A.rows();
    ny = C.rows();
    P.resize(nx, nx);
    X.resize(nx);
    // Q = Eigen::MatrixXd::Identity(nx, nx);
    // R = Eigen::MatrixXd::Identity(ny, ny);
    K.resize(nx, ny);
    Y.resize(ny, ny);
    I = Eigen::MatrixXd::Identity(nx, nx);
  }

  /**
   * @brief 初始化状态变量X和协方差矩阵P(初始化为0矩阵)
   */
  void reset(Eigen::VectorXd& x0) {
    X = x0;
    P = Eigen::MatrixXd::Zero(nx, nx);  // P矩阵初始化为0矩阵
  }
  /**
   * @brief 估计系统状态:欧拉法离散状态方程
   * @param u 控制输入变量
   * @param y 测量输入
   * @param dt 时间步长
   * @return 估计值
   */
  Eigen::VectorXd step(Eigen::VectorXd& u, Eigen::VectorXd& y, double dt) {
    // Phi矩阵：phi = (I+dt*A)
    Phi = I + dt * A;
    G = dt * B;

    // 预测
    X = Phi * X + G * u;
    P = Phi * P * Phi.transpose() + Q;  // 更新协方差矩阵
    // 更新
    Y = C * P * C.transpose() + R;
    K = P * C.transpose() * Y.inverse();  // 卡尔曼增益
    // 修正
    X += K * (y - C * X);  // 重复利用变量X
    P = (I - K * C) * P;   // 更新协方差矩阵

    return X;
  }

 private:
  // A B C分别是状态矩阵，控制输入矩阵和观测矩阵
  Eigen::MatrixXd A, B, C;
  // 协方差矩阵
  Eigen::MatrixXd Q, R;
  // 辅助矩阵
  Eigen::MatrixXd P, K, Y, I, Phi, G;
  // 状态变量
  Eigen::VectorXd X;
  // 矩阵大小
  int nx, ny;
};

#endif