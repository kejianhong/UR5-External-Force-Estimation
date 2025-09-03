#include "UR5Simulation/observers.h"

#include <vector>

#include "ExtObserverLib_v3/filtered_dyn_observer_rnea.h"
#include "ExtObserverLib_v3/kalman_filter_observer_rnea.h"
#include "ExtObserverLib_v3/kalman_filter_observer_zero_order_rnea.h"
#include "ExtObserverLib_v3/momentum_observer_rnea.h"
#include "ExtObserverLib_v3/nonlinear_disturbance_observer_rnea.h"
#include "ExtObserverLib_v3/sliding_mode_observer_rnea.h"

std::shared_ptr<ExternalObserverRnea> getObserver(RobotDynamicsRnea* robot, const int& observerType, const double& timeStep, const std::string& paramType) {
  const int dof = robot->jointNo();
#ifdef MOMENTUM_OBSERVER_RNEA_H
  Vector k(dof);
  if (paramType == "normal") {
    // 正常
    k << 10, 4, 5, 10, 20, 20;
  } else if (paramType == "small") {
    // 偏小
    k << 2.5, 1, 1.25, 2.5, 5, 5;
  } else if (paramType == "big") {
    // 偏大
    k << 30, 12, 15, 30, 60, 60;
  }
  std::shared_ptr<MomentumObserverRnea> m_observer(new MomentumObserverRnea(robot, k));
#endif

#ifdef NONLINEAR_DISTURBANCE_OBSERVER_RNEA_H
  /**
   * @param sigma M(q)<=sigma*I
   * @param xeta ||d(M(q))/dt||<=xeta
   * @param beta 最小收敛速
   * @param Y 类似加权矩阵，Y=0.5(xeta+2*beta*sigma)I
   */
  double sigma, xeta, beta;
  if (paramType == "normal") {
    // 正常
    sigma = 50, xeta = 50, beta = 50;
  } else if (paramType == "small") {
    // 偏小
    sigma = 40, xeta = 40, beta = 40;
  } else if (paramType == "big") {
    // 偏大
    sigma = 100, xeta = 100, beta = 100;
  }
  std::shared_ptr<DisturbanceObserverRnea> d_observer(new DisturbanceObserverRnea(robot, sigma, xeta, beta));
#endif

#ifdef SLIDING_MODE_OBSERVER_RNEA_H
  Vector T1(dof), T2(dof), S1(dof), S2(dof);
  if (paramType == "normal") {
    // 正常
    S1 << 30, 20, 20, 10, 10, 10;
    S2 << 20, 10, 10, 10, 5, 5;  // 当S2取0时,退化为2阶滑膜控制
  } else if (paramType == "small") {
    // 偏小
    S1 << 6, 4, 4, 2.5, 2.5, 2.5;
    S2 << 4, 2.5, 2.5, 2.5, 1, 1;  // 当S2取0时,退化为2阶滑膜控制
  } else if (paramType == "big") {
    // 偏大
    S1 << 150, 100, 100, 50, 50, 50;
    S2 << 100, 50, 50, 50, 25, 25;
  }
  for (int i = 0; i < dof; i++) {
    T1(i) = 2 * sqrt(S1(i));
    T2(i) = 2 * sqrt(S2(i));
  }
  std::shared_ptr<SlidingModeObserverRnea> sm_observer(new SlidingModeObserverRnea(robot, T1, S1, T2, S2));
#endif

#ifdef FILTERED_DYN_OBSERVER_RNEA_H
  double cutOff;
  if (paramType == "normal") {
    // 正常
    cutOff = 5;
  } else if (paramType == "small") {
    // 偏小
    cutOff = 2;
  } else if (paramType == "big") {
    // 偏大
    cutOff = 20;
  }
  std::shared_ptr<FilterDynObserverRnea> fd_observer(new FilterDynObserverRnea(robot, cutOff, timeStep));
#endif

#ifdef KALMAN_FILTER_OBSERVER_RNEA_H
  // 需要调节的就只有Q1和R1
  Matrix kalmanH1 = Matrix::Identity(6, 6);
  Matrix kalmanS1 = Matrix::Zero(6, 6);
  Matrix kalmanQ1 = Matrix::Zero(12, 12);
  Matrix kalmanR1 = Matrix::Identity(6, 6);

  if (paramType == "normal") {
    // 噪声水平一致:效果最好
    // 计算噪声越小表示更相信计算值(在仿真中,计算值更可靠,计算噪声越小,估计噪声越小)
    //前面６个元素是摩擦力的协方差比较大，越小收敛越快(仿真中没有摩擦力量,所以设置为0)
    // kalmanQ1(i, i) = 0;
    //后面六个元素是外力的扰动协方差,越小收敛越慢
    kalmanQ1(6, 6) = kalmanQ1(7, 7) = kalmanQ1(8, 8) = kalmanQ1(9, 9) = 1;
    kalmanQ1(10, 10) = kalmanQ1(11, 11) = 5;
    // 测量噪声越小,表示更相信测量值(在仿真中,测量值不可靠,测量噪声越小,估计噪声越大)
    kalmanR1(0, 0) = kalmanR1(1, 1) = kalmanR1(2, 2) = 1;
    kalmanR1(3, 3) = kalmanR1(4, 4) = kalmanR1(5, 5) = 0.5;
  } else if (paramType == "small") {
    // 更相信测量值:噪声
    // 计算噪声越小表示更相信计算值(在仿真中,计算值更可靠,计算噪声越小,估计噪声越小)
    //前面６个元素是摩擦力的协方差比较大，越小收敛越快(仿真中没有摩擦力量,所以设置为0)
    // kalmanQ1(i, i) = 0;
    //后面六个元素是外力的扰动协方差,越小收敛越慢
    kalmanQ1(6, 6) = kalmanQ1(7, 7) = kalmanQ1(8, 8) = kalmanQ1(9, 9) = 10;
    kalmanQ1(10, 10) = kalmanQ1(11, 11) = 50;
    // 测量噪声越小,表示更相信测量值(在仿真中,测量值不可靠,测量噪声越小,估计噪声越大)
    kalmanR1(0, 0) = kalmanR1(1, 1) = kalmanR1(2, 2) = 0.1;
    kalmanR1(3, 3) = kalmanR1(4, 4) = kalmanR1(5, 5) = 0.05;
  } else if (paramType == "big") {
    // 更相信计算值:相位延迟
    // 计算噪声越小表示更相信计算值(在仿真中,计算值更可靠,计算噪声越小,估计噪声越小)
    //前面６个元素是摩擦力的协方差比较大，越小收敛越快(仿真中没有摩擦力量,所以设置为0)
    // kalmanQ1(i, i) = 0;
    //后面六个元素是外力的扰动协方差,越小收敛越慢
    kalmanQ1(6, 6) = kalmanQ1(7, 7) = kalmanQ1(8, 8) = kalmanQ1(9, 9) = 0.1;
    kalmanQ1(10, 10) = kalmanQ1(11, 11) = 0.5;
    // 测量噪声越小,表示更相信测量值(在仿真中,测量值不可靠,测量噪声越小,估计噪声越大)
    kalmanR1(0, 0) = kalmanR1(1, 1) = kalmanR1(2, 2) = 10;
    kalmanR1(3, 3) = kalmanR1(4, 4) = kalmanR1(5, 5) = 5;
  }
  std::shared_ptr<KalmanObserverRnea> kalman_observer(new KalmanObserverRnea(robot, kalmanH1, kalmanS1, kalmanQ1, kalmanR1));
#endif

#ifdef KALMAN_FILTER_OBSERVER_ZERO_ORDER_RNEA_H
  Matrix kalmanH2 = Matrix::Identity(6, 6);
  Matrix kalmanS2 = Matrix::Zero(6, 6);
  Matrix kalmanQ2 = Matrix::Zero(12, 12);
  Matrix kalmanR2 = Matrix::Identity(6, 6);

  if (paramType == "normal") {
    // 噪声水平一致:效果最好
    // 计算噪声越小表示更相信计算值(在仿真中,计算值更可靠,计算噪声越小,估计噪声越小)
    // 前面６个元素是摩擦力的协方差比较大，越小收敛越快(仿真中没有摩擦力量,所以设置为0)
    // kalmanQ2(i, i) = 0;
    // 后面六个元素是外力的扰动协方差,越小收敛越慢
    kalmanQ2(6, 6) = kalmanQ2(7, 7) = kalmanQ2(8, 8) = 1000;
    kalmanQ2(9, 9) = kalmanQ2(10, 10) = kalmanQ2(11, 11) = 1000;
    // // 测量噪声越小,表示更相信测量值(在仿真中,测量值不可靠,测量噪声越小,估计噪声越大)
    kalmanR2(0, 0) = kalmanR2(1, 1) = kalmanR2(2, 2) = 0.5;
    kalmanR2(3, 3) = kalmanR2(4, 4) = kalmanR2(5, 5) = 0.1;
  } else if (paramType == "small") {
    // 更相信测量值:噪声
    // 计算噪声越小表示更相信计算值(在仿真中,计算值更可靠,计算噪声越小,估计噪声越小)
    // 前面６个元素是摩擦力的协方差比较大，越小收敛越快(仿真中没有摩擦力量,所以设置为0)
    // kalmanQ2(i, i) = 0;
    // 后面六个元素是外力的扰动协方差,越小收敛越慢
    kalmanQ2(6, 6) = kalmanQ2(7, 7) = kalmanQ2(8, 8) = 5000;
    kalmanQ2(9, 9) = kalmanQ2(10, 10) = kalmanQ2(11, 11) = 5000;
    // 测量噪声越小,表示更相信测量值(在仿真中,测量值不可靠,测量噪声越小,估计噪声越大)
    kalmanR2(0, 0) = kalmanR2(1, 1) = kalmanR2(2, 2) = 0.1;
    kalmanR2(3, 3) = kalmanR2(4, 4) = kalmanR2(5, 5) = 0.02;
  } else if (paramType == "big") {
    // 更相信计算值:相位延迟
    // 计算噪声越小表示更相信计算值(在仿真中,计算值更可靠,计算噪声越小,估计噪声越小)
    //前面６个元素是摩擦力的协方差比较大，越小收敛越快(仿真中没有摩擦力量,所以设置为0)
    // kalmanQ2(i, i) = 0;
    //后面六个元素是外力的扰动协方差,越小收敛越慢
    kalmanQ2(6, 6) = kalmanQ2(7, 7) = kalmanQ2(8, 8) = 100;
    kalmanQ2(9, 9) = kalmanQ2(10, 10) = kalmanQ2(11, 11) = 100;
    // 测量噪声越小,表示更相信测量值(在仿真中,测量值不可靠,测量噪声越小,估计噪声越大)
    kalmanR2(0, 0) = kalmanR2(1, 1) = kalmanR2(2, 2) = 5;
    kalmanR2(3, 3) = kalmanR2(4, 4) = kalmanR2(5, 5) = 1;
  }
  std::shared_ptr<KalmanObserverZeroOrderRnea> kalman_zero_order_observer(new KalmanObserverZeroOrderRnea(robot, kalmanH2, kalmanS2, kalmanQ2, kalmanR2));
#endif
  std::vector<std::shared_ptr<ExternalObserverRnea>> observers = {m_observer, d_observer, sm_observer, fd_observer, kalman_observer, kalman_zero_order_observer};

  return observers[observerType];
}