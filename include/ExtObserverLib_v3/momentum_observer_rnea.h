#ifndef MOMENTUM_OBSERVER_RNEA_H
#define MOMENTUM_OBSERVER_RNEA_H

#include"external_observer.h"

#define ID_MomentumObserverRnea 1

/**
 * @brief 对于广义动量观测器，当权重矩阵是对角矩阵时，等价与一个ｎ阶低通滤波器，所以可以针对每个关节单独调节，权重越大，估计效果越好，但是太大会导致震荡
 */
class MomentumObserverRnea: public ExternalObserverRnea {
public:
    /**
     * @brief 构造函数
     * @param rd 机器人实例指针
     * @param k 关节增益向量
     */
    MomentumObserverRnea(RobotDynamicsRnea* rd, Vector& k);

    /**
     * @brief 基于广义动量的外部力矩观测器
     * @param q 关节角度
     * @param qd 关节速度
     * @param tau 考虑重力，外力，摩擦力的关节力矩向量，即 tau = M(q)*qdd + C(q,qd)*qd + F(qd) + G(q) + tau_ext
     */
    Vector getExternalTorque(Vector&q, Vector& qd, Vector& tau, double dt) override;

protected:
    // 累加因子
    Vector sum, r, zero;
    // 临时变量
    Vector p, beta, torque, t_prev;
    // gain
    Vector ko;
};

#endif // MOMENTUM_OBSERVER_RNEA_H