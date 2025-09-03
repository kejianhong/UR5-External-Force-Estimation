#ifndef KALMAN_FILTER_OBSERVER_RNEA_H
#define KALMAN_FILTER_OBSERVER_RNEA_H

#include <memory>

#include "external_observer.h"
#include "kalman_filter_euler.h"
#define ID_KalmanObserverRnea 21
/**
 * @brief 基于欧拉法的卡尔曼滤波外部力矩观测器
 */

class KalmanObserverRnea : public ExternalObserverRnea {
 public:
  /**
   * @brief 构造函数
   * @param rd 机器人实例指针
   * @param j 雅可比矩阵的转置(可取单位矩阵)
   * @param s 扰动矩阵(一般取0矩阵)
   * @param q 过程噪声协方差矩阵：Q[Qc(摩擦力相关的协方差) 0; 0 Qf(外力扰动的协方差)]
   * @param r 观测噪声的协方差矩阵：测量广义动量的速度噪声协方差
   */
  KalmanObserverRnea(RobotDynamicsRnea* rd, Matrix& j, Matrix& s, Matrix& q, Matrix& r);

  // 析构函数
  ~KalmanObserverRnea() {
    filter = nullptr;
  }

  /**
   * @brief 外力估计函数
   * @param q 关节角度
   * @param qd 关节速度
   * @param tau 点电机力矩
   * @param dt 时间步长
   */
  Vector getExternalTorque(Vector& q, Vector& qd, Vector& tau, double dt) override;

 private:
  // 扰动观测矩阵
  Matrix J;
  // 状态向量，输入控制向量，广义动量，零向量
  Vector X, u, p, zero;
  // 卡尔曼滤波器实例
  std::shared_ptr<KalmanFilter> filter;
};

#endif