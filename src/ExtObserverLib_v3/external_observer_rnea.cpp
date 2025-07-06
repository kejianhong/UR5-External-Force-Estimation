#include"ExtObserverLib_v3/external_observer.h"

#define DELTA_INFT 1E-7

/** @brief 构造函数 */
RobotDynamicsRnea::RobotDynamicsRnea():
_qext(Vector(1)),
_p0(Vector(1)),
_zero(Vector(1)),
_sum(Vector(1)),
_M(Matrix(1,1)),
delta(DELTA_INFT)
{}


Vector RobotDynamicsRnea::tranCqd(Vector& q, Vector& qd){
    // 只调用一次
    // 根据机器人自由度初始化各个临时变量
    int N = jointNo();
    _zero.resize(N);
    _zero.setZero();
    // p0 = M(q)*qd + C(q,0)*0 + F(0) +G(q) = RNEA(q, 0, qd, 0)
    _p0.resize(N);
    _p0 = rnea(q, _zero, qd, 0);
    //
    _sum.resize(N);
    _sum.setZero();
    
    for(int i = 0; i<N; i++){
        _qext = q;
        _qext(i) += delta; // 微分时角度增量
        _sum += (rnea(_qext, _zero, qd, 0) - _p0) * qd(i) / delta; // M'*q',微分转差分，单个关节角度累计
    }
    _sum -= rnea(q, qd, _zero, 0); // M'*q' - C*q'

    return _sum;
}


Matrix RobotDynamicsRnea::getM(Vector& q){
    // 只调用一次
    int N = jointNo();
    _zero.resize(N);
    _zero.setZero();
    _M.resize(N,N);
    _qext.resize(N);

    // 使用单位向量vj选中惯性矩阵的每一列M(,j) = RNEA(q,0,vj,0)
    for(int i=0; i<N; i++){
        _qext.setZero();
        _qext(i) = 1; // vj
        _p0 = rnea(q,_zero,_qext, 0); // 利用_p0保存M(,j)
        for(int j=0; j<N; j++){
            _M(j,i) = _p0(j);
        }
    }

    return _M;
}