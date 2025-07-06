#ifndef NONLINEAR_DISTURBANCE_OBSERVER_RNEA_H
#define NONLINEAR_DISTURBANCE_OBSERVER_RNEA_H

#include"external_observer.h"

#define ID_DisturbanceObserverRnea 2
class DisturbanceObserverRnea: public ExternalObserverRnea {
public:
    /**
     * @brief 构造函数
     * @param rd 机器人实例指针
     * @param sigma M(q)<=sigma*I
     * @param xeta ||d(M(q))/dt||<=xeta
     * @param beta 最小收敛速
     */
    DisturbanceObserverRnea(RobotDynamicsRnea* rd, double sigma, double xeta, double beta);
    
    /**
     * @brief 基于非线性扰动观测器的外部力矩观测器
     * @param q 关节角度
     * @param qd 关节速度
     * @param tau 电机读出来的力矩
     * @param dt 时间步长
     */
    Vector getExternalTorque(Vector& q, Vector& qd, Vector& tau, double dt) override;

private:
    // 临时变量
    Matrix Y, L, I;
    Matrix lft, rht;
    Vector phi, z, torque, torqueEst,zeros;
};

#endif