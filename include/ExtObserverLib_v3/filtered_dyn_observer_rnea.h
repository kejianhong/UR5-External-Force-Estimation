#ifndef FILTERED_DYN_OBSERVER_RNEA_H
#define FILTERED_DYN_OBSERVER_RNEA_H

#include "external_observer.h"
#include "iir_filter.h"

#define ID_FilterDynObserverRnea 22

class FilterDynObserverRnea : public ExternalObserverRnea {
 public:
  /**
   * @brief 构造函数
   * @param rd 机器人实例指针
   * @param cutOff 数字截止角频率
   * @param sampTime 采样周期
   */
  FilterDynObserverRnea(RobotDynamicsRnea* rd, double cutOff, double sampTime);

  /**
   * @brief 外力估计函数
   * @param q 关节位置
   * @param qd 关节速度
   * @param tau 电机力矩
   * @param dt 采样时间/采样周期
   */
  Vector getExternalTorque(Vector& q, Vector& qd, Vector& tau, double dt) override;

 private:
  FilterF1 f1;
  FilterF2 f2;
  Vector p, tauExt, zeros, u;
};

#endif