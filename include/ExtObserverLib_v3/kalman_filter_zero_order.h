#ifndef KALMAN_FILTER_ZERO_ORDER_RNEA_H
#define KALMAN_FILTER_ZERO_ORDER_RNEA_H

#include <eigen3/Eigen/Geometry>

class KalmanFilterZeroOrder {
 public:
  /**
   * @brief 构造函数
   * @param a 状态矩阵
   * @param b 控制输入矩阵
   * @param c 观测矩阵
   */
  KalmanFilterZeroOrder(Eigen::MatrixXd& a, Eigen::MatrixXd& b, Eigen::MatrixXd& c, Eigen::MatrixXd& q, Eigen::MatrixXd& r) : Ad(a), Bd(b), Cd(c), Rd(Eigen::MatrixXd(1, 1)), Qd(Eigen::MatrixXd(1, 1)), Rupd(Eigen::MatrixXd(1, 1)) {
    na = a.rows();
    nb = b.cols();
    nc = c.rows();
    P.resize(na, na);  // 预测协方差矩阵(predict)
    X.resize(na);      // 状态向量
    Qd = q;            // 离散化之前的预测过程的协方差矩阵
    R = r;             // 离散化之前的测量过程的速度协方差矩阵
    Rupd = r;          // 离散化之前的测量过程的动量协方差矩阵
    Rd = r;            // 离散化之后的测量过程的动量协方差矩阵　Rd = Rupd/Ts
    K.resize(na, nc);  // 增益矩阵
    Y.resize(nc, nc);  //
    I = Eigen::MatrixXd::Identity(na, na);

    // AB = [A B; 0 0], ABd是为了计算AB离散化后对应的矩阵exp(AB*Ts)
    AB.resize(na + nb, na + nb);
    AB.setZero();
    AB.block(0, 0, na, na) = a;
    AB.block(0, na, na, nb) = b;
    ABd.resize(na + nb, na + nb);

    // AQ = [A Q; 0, -A], AQd是为了计算Q离散化后的矩阵
    AQ.resize(na + na, na + na);
    AQ.block(0, 0, na, na) = a;
    AQ.block(0, na, na, na) = Qd;  //
    AQ.block(na, na, na, na) = -a.transpose();
    AQd.resize(na + na, na + na);
  }

  /**
   * @brief 初始化系统的状态变量X和协方差矩阵(初始化为0矩阵)
   * @param x0 x0=[p;f];
   */
  void reset(Eigen::VectorXd& x0) {
    X = x0;
    P = Eigen::MatrixXd::Zero(na, na);  // 预测协方差矩阵，初始化为0矩阵
  }

  /**
   * @brief 更新测量噪声协方差矩阵Rc = M*R(vel)*M'
   */
  void updateR(Eigen::MatrixXd& m) {
    Rupd = m * R * m.transpose();
  }

  /**
   * @brief 卡尔曼滤波估计的过程
   * @param u 控制输入向量
   * @param y 输出向量
   * @param dt 时间步长
   */
  Eigen::VectorXd step(Eigen::VectorXd& u, Eigen::VectorXd& y, double dt);

 private:
  // 系统矩阵
  Eigen::MatrixXd Ad, Bd, Cd;
  // 协方差矩阵
  Eigen::MatrixXd R, Rd, Qd, Rupd;
  // 块状矩阵
  Eigen::MatrixXd AB, AQ, ABd, AQd;
  // 辅助矩阵
  Eigen::MatrixXd P, K, Y, I;
  // 系统变量
  Eigen::VectorXd X;
  // 矩阵大小
  int na, nb, nc;

  // 泰勒展开计算指数矩阵
  Eigen::MatrixXd exponential(Eigen::MatrixXd& m, double dt);
  // 离散化矩阵
  void makeDiscrete(double dt);
};

#endif