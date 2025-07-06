#include "ExtObserverLib_v3/kalman_filter_observer_zero_order_rnea.h"

/**
 * @brief 构造函数
 * @param rd 机器人实例指针
 * @param j 机器人雅可比矩阵(可以选择为单位矩阵)
 * @param s 扰动矩阵(一般为0矩阵)
 * @param q 过程噪声协方差矩阵：Q[Qc(摩擦力相关的协方差) 0; 0 Qf(外力扰动的协方差)]
 * @param r 观测噪声的协方差矩阵：测量广义动量的速度噪声协方差
 */
KalmanObserverZeroOrderRnea::KalmanObserverZeroOrderRnea(RobotDynamicsRnea* rd, Matrix& j, Matrix& s, Matrix& q, Matrix& r):
ExternalObserverRnea(rd, ID_KalmanObserverZeroOrderRnea),
J(j), M(Matrix(jointNo, jointNo)),
X(Vector(2*jointNo)),// 对于6自由度机器人,X=[p;f]，p和f维度一样
u(Vector(jointNo)),
p(Vector(jointNo)),
zero(Vector::Zero(jointNo))
{
    Matrix A(2*jointNo, 2*jointNo), B(2*jointNo, jointNo), C(jointNo, 2*jointNo);
    A.setZero();
    B.setZero();
    C.setZero();
    // A[0, J; 0, 0] B[I; 0] C[I 0]
    A.block(0,jointNo, jointNo, jointNo) = j;
    A.block(jointNo, jointNo, jointNo, jointNo) = s;
    for(int i=0; i<jointNo; i++){
        B(i,i)=1;
        C(i,i)=1;
    }
    // 指针, A B C是系统矩阵
    filter = std::make_shared<KalmanFilterZeroOrder>(KalmanFilterZeroOrder(A, B, C, q,r));
}

Vector KalmanObserverZeroOrderRnea::getExternalTorque(Vector&q, Vector&qd, Vector&tau, double dt){
    M = dyn->getM(q);// 惯性矩阵
    p = M*qd;// 广义动量
    u = tau-dyn->rnea(q, zero, zero, GRAVITY) -dyn->getFriction(qd) + dyn->tranCqd(q,qd);// 控制变量

    // 将速度协方差矩阵转化为广义动量协方差矩阵
    filter->updateR(M);

    if(isRun){
        X = filter->step(u, p, dt);
    }else{
        X.setZero();
        for(int i=0; i<jointNo; i++)
            X(i) = p(i);// 初始化为X0=[p0;0]
        filter->reset(X);
        isRun = true;
    }
    p = J*X.block(jointNo,0,jointNo,1);
    return p;
}