#ifndef SLIDING_MODE_OBSERVER_RNEA_H
#define SLIDING_MODE_OBSERVER_RNEA_H

#include"external_observer.h"

#define BIGR 50 // 使用tanh替代sign
#define ID_SlidingModeObserverRnea 3

class SlidingModeObserverRnea: public ExternalObserverRnea {
public:
    /**
     * @brief 构造函数
     * @param rd 机器人实例指针
     * @param t1 加权对叫矩阵的对角元素
     * @param s1 加权对叫矩阵的对角元素
     * @param t2 加权对叫矩阵的对角元素
     * @param s2 加权对叫矩阵的对角元素
     */
    SlidingModeObserverRnea(RobotDynamicsRnea* rd, Vector& t1, Vector& s1, Vector& t2, Vector& s2);

    /**
     * @brief 力矩估计函数
     * @param q 关节位置
     * @param qd 关节速度
     * @param tau 电机力矩
     * @param dt 时间步长
     */
    Vector getExternalTorque(Vector& q, Vector& qd, Vector& tau, double dt) override;

private:
    Vector sigma, p_hat, tauExt;
    Vector p, dispp, dp_hat, u, dsigma, zeros;
    Vector T1, T2, S1, S2;
};

#endif