#ifndef UR5SIMULATIONMODEL_H
#define UR5SIMULATIONMODEL_H

#include "ExtObserverLib_v3/external_observer.h"

typedef Eigen::MatrixXd Matrix;
typedef Eigen::VectorXd Vector;

const int UR5DOF = 6;
const int baseParamSet1Size = 40;
const int baseParamSet2Size = 24;

class UR5 : public RobotDynamicsRnea
{
public:
    /**
     * @brief 默认构造函数
     */
    UR5();
    /**
     * @brief RNEA实现，继承RobotDynamicsRnea
     */
    Vector rnea(Vector &q, Vector &qd, Vector &qdd, double g=0) override;
    /**
     * @brief friction，继承RobotDynamicsBase
     */
    Vector getFriction(Vector &qd) override;
    /**
     * @brief joint degree，继承RobotDynamicsBase
     */
    int jointNo() override;
    /**
     * @brief Jacobian matrix:TCP相对基坐标系的雅可比矩阵在TCP坐标系中的表示
     */
    Matrix getVelocityJacobianEndEffector(const Vector &q) override;
    /**
     * @brief Jacobian matrix:TCP相对基坐标系的雅可比矩阵在基坐标系中的表示
     */
    Matrix getVelocityJacobianBase(const Vector &q) override;
    /**
     * @brief 力变换矩阵:将力从传感器坐标系变换到TCP坐标系
     */
    Matrix getForceTransformMatrix() override;
private:
    /**
     * @brief 读取基本惯性参数，通过辨识得到，hardcode，前40个是与惯性参数相关，后18个与摩擦力相关
     * @return 返回基本惯性参数向量
     */
    void initBaseParameterSet();

    /**
     * @brief 回归矩阵计算
     * @param q 关节位置
     * @param qd 关节速度
     * @param qdd 关节加速度
     * @param g 重力常量
     */
    Matrix regressorMatrix(Vector &q, Vector &qd, Vector &qdd, double g);

    Vector tau, fric, baseParmSet1, baseParmSet2;
    Matrix jacobian, forceTransformMatrix;
};

#endif