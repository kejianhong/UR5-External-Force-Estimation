#ifndef IMPEDANCECONTROLEXPFAKEWRENCH_H
#define IMPEDANCECONTROLEXPFAKEWRENCH_H

#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <eigen3/Eigen/Geometry>
#include <memory>
#include <sstream>
#include <tuple>
#include <vector>

#include "Log/logger.h"
#include "UR5Simulation/UR5SimulationModel.h"

using namespace ur_rtde;

class ImpedanceControl {
 private:
  // 导纳控制参数
  const Vector initM, initD, initK;
  Vector M, D, K;
  // 观测器参数设置
  const std::string parameterType;
  // 控制频率
  int frequency;
  // 控制周期
  double dt;
  // 观测器类型
  int observerType;
  // 连接机器人
  RTDEControlInterface rtde_control;
  RTDEReceiveInterface rtde_receive;
  // 机器人指针
  std::shared_ptr<RobotDynamicsRnea> robot;
  // 观测器指针
  std::shared_ptr<ExternalObserverRnea> observer;
  // 安全设置
  double maxVel, maxAcc;
  // servoJ参数设置
  double acceleration;
  double velocity;
  double lookaheadTime;
  double gain;
  // 运行标志
  static bool runtimeFlag;
  // 阻抗控制状态标志:0表示无外力状态,1表示外力施加状态
  int state = 0;
  // 外力
  Vector tcpTauExt;
  // 误差向量
  Vector error, derror, dderror;
  // 平衡位姿
  Vector equilibriumTranslation;             // 3*1向量
  Eigen::Quaterniond equilibriumQuaternion;  // 四元数
  // 实时位姿
  Vector currTranslation;             // 3*1向量
  Eigen::Quaterniond currQuaternion;  // 四元数
  // 电流转换成力矩常数
  std::vector<double> motorGain = {12.625, 12.625, 12.625, 9.3122, 9.3122, 9.3122};
  std::stringstream ss;
  // 导纳控制计算
  void computeImpedance();
  // 安全设置,限制速度,加速度
  void limitVelocity();
  // 将UR5的六维位姿(3*1位置 + 3*1旋转向量)转化为(3*1位置 + 四元数表示)
  static std::tuple<Vector, Eigen::Quaterniond> getTcpTransformPose(const std::vector<double>&);
  // 减速过程更新导纳控制的三个矩阵M,D,K
  void updateParamMatrix();
  // 复原参数矩阵
  void resetParamMatrix();
  // 信号函数
  static void signalCallbackFunc(int val) {
    LOG(WARNING, "User interrepts the process.");
    runtimeFlag = false;
  }

 public:
  // 显式构造函数
  explicit ImpedanceControl(std::vector<double>& Ma, std::vector<double>& Da, std::vector<double>& Ka, int fre, int oType, const std::string& ip, const std::string& parameterType, std::shared_ptr<RobotDynamicsRnea> rd, double mAcc = 0.5, double mVel = 0.5, double acc = 0.5, double vel = 0.5, double lT = 0.1, double gain = 300);
  ~ImpedanceControl();
  void runImpedanceController(const std::vector<int>& mask);
  void init();
  void clearLog();
};
#endif