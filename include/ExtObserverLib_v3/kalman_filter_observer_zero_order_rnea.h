#ifndef KALMAN_FILTER_OBSERVER_ZERO_ORDER_RNEA_H
#define KALMAN_FILTER_OBSERVER_ZERO_ORDER_RNEA_H

/**
 * @brief 零阶保持器法实现基于卡尔曼滤波的外部力矩观测器
 */
#include <memory>

#include "external_observer.h"
#include "kalman_filter_zero_order.h"

#define ID_KalmanObserverZeroOrderRnea 27

class KalmanObserverZeroOrderRnea : public ExternalObserverRnea {
 public:
  /**
   * @brief 构造函数
   * @param rd 机器人实例指针
   * @param j 机器人雅可比矩阵(可以选择为单位矩阵)
   * @param s 扰动矩阵(一般为0矩阵)
   * @param q 过程噪声协方差矩阵：Q[Qc(摩擦力相关的协方差) 0; 0 Qf(外力扰动的协方差)]
   * @param r 观测噪声的协方差矩阵：测量广义动量的速度噪声协方差
   */
  KalmanObserverZeroOrderRnea(RobotDynamicsRnea* rd, Matrix& j, Matrix& s, Matrix& q, Matrix& r);

  // 析构函数
  ~KalmanObserverZeroOrderRnea() {
    filter = nullptr;
  }

  /**
   * @brief 力矩估计函数
   * @param q 角度
   * @param qd 速度
   * @param tau 电机力矩
   * @param dt 时间步长
   */
  Vector getExternalTorque(Vector& q, Vector& qd, Vector& tau, double dt) override;

 private:
  // J雅可比矩阵，M惯性矩阵
  Matrix J, M;
  // 状态向量，控制输入向量，广义动量，零向量
  Vector X, u, p, zero;
  // 卡尔曼滤波实例指针
  std::shared_ptr<KalmanFilterZeroOrder> filter;
};
#endif