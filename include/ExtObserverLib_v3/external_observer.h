/**
 * @file external_observer.h
 * @brief Abstract classes for a robot and external torque observer
 * 采用牛顿－欧拉法，通过数值代替进行计算RNEA(q,dq,ddq,g)
 */
#ifndef EXTERNAL_OBSERVER_H
#define EXTERNAL_OBSERVER_H
#include<eigen3/Eigen/Geometry>

/** @brief 重力常量，正负号待定 */
#define GRAVITY -9.81

/** @brief 动态矩阵和向量的类型别名，方面调用 */
typedef Eigen::MatrixXd Matrix;
typedef Eigen::VectorXd Vector;

/**
 * @brief RNEA(q,qd,qdd,g)算法的具体实现，包含惯性矩阵M的计算，科氏力C^T * qd的计算
 */
class RobotDynamicsRnea {
public:
    /**
     * @brief 构造函数
     */
    RobotDynamicsRnea();

    /**
     * @brief 牛顿－欧拉法计算，用于数值替代，RNEA(q,qd,qdd,g=0) = M(q)*qd + C(q,qd)*qd + G(q)，不包含摩擦力
     * @param q 关节位置
     * @param qd 关节速度
     * @param qdd 关节加速度
     * @param g 重力常数
     */
    virtual Vector rnea(Vector& q, Vector& qd, Vector& qdd, double g = 0) = 0;

    /**
     * @brief 关节摩擦力虚函数
     * @param qd 关节速度向量
     * @return 摩擦力
     */
    virtual Vector getFriction(Vector& qd) = 0;

    /**
     * @brief 可动关节数量，即自由度
     * @return 自由度
     */
    virtual int jointNo() = 0;

    /**
     * @brief TCP相对基坐标系的雅可比矩阵在TCP坐标系中的表示
     * @return TCP相对基坐标系的雅可比矩阵在TCP坐标系中的表示
     */
    virtual Matrix getVelocityJacobianEndEffector(const Vector &q) = 0;

    /**
     * @brief TCP相对基坐标系的雅可比矩阵在基坐标系中的表示
     * @return TCP相对基坐标系的雅可比矩阵在基坐标系中的表示
     */
    virtual Matrix getVelocityJacobianBase(const Vector &q) = 0;

    /**
     * @brief 计算从传感器坐标系变换到基坐标系下的六维力变换矩阵
     * @param pose = [位置向量(3*1); 姿态(3*1旋转矢量)]
     * @return 力变换矩阵
     */
    virtual Matrix getForceTransformMatrix() = 0;
    /**
     * @brief 设置角度步长用于微分计算(derivative estimation)
     * @param d 角度步长，默认值是1E-7
     */
    void setDelta(double d) {delta = d;};

    /**
     * @brief 计算C^T * qd, Equation(36~37)，依赖rnea虚函数rnea实现
     * @param q 关节位置
     * @param qd 关节速度
     */
    Vector tranCqd(Vector& q, Vector& qd);

    /**
     * @brief 计算惯性矩阵，使用单位向量vj选中惯性矩阵的每一列M(,j) = RNEA(q,0,vj,0)，依赖rnea虚函数的实现
     * @param q 关节位置
     */
    Matrix getM(Vector& q);

protected:
    /** 
     * @brief 临时变量，减少内存开支
     * @param _qext 辅助向量，哪里需要哪里搬
     * @param _p0 初始广义动量
     * @param _zero 零矩阵／向量
     * @param _sum 求和
     */
    Vector _qext, _p0, _zero, _sum;
    /** @brief _M */
    Matrix _M;
    /** @brief 角度步长*/
    double delta;

}; // RobotDynamicsRnea



//////////////////////////////////////////////////////////
/**
 * @brief 外部力矩观测器，包含rnea算法指针和观测器类型标志符objType
 */
class ExternalObserverRnea {
public:
    /**
     * @brief 构造函数
     */
    ExternalObserverRnea(RobotDynamicsRnea *rd, int type): dyn(rd) {
        jointNo = rd->jointNo(); // 基类RobotDynamicsBase的虚函数
        objType = type; // 观测器类型选择
        isRun = false;// 初始化为false
        }

    /**
     * @brief 外力估计函数
     * @param q 关节位置
     * @param dq 关节速度
     * @param tau 关节力矩，包含摩擦力，重力，外力引起的力增量
     * @param dt time step from the previous call
     * @return 返回关节外力估计向量
     */
    virtual Vector getExternalTorque(Vector& q, Vector& qd, Vector& tau, double dt) = 0;
protected:
    /** @brief RNEA类指针 */
    RobotDynamicsRnea *dyn;
    int jointNo; // 关节数
    int objType; // 观测器类型
    bool isRun;  // 初始化观测器时更改其状态
}; // ExternalObserverRnea
#endif