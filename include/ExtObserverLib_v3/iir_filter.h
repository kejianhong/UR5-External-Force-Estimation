#ifndef IIR_FILTER_H
#define IIR_FILTER_H

#include <cmath>
#include <eigen3/Eigen/Geometry>

typedef Eigen::VectorXd Vector;

class FilterIIR {
 public:
  // 计算数字滤波器的系数
  virtual void update(double cutOff, double sampTime) = 0;
};

// H(s) = w/(s+w)
class FilterF1 : public FilterIIR {
 public:
  /**
   * @brief 构造函数
   * @param cutOff 数字截止角频率
   * @param sampTime 采样时间
   * @param N 自由度
   */
  FilterF1(double cutOff, double sampTime, int N) : FilterIIR(), x1(Vector(N)), y1(Vector(N)) {
    update(cutOff, sampTime);
  }

  /**
   * @brief 滤波
   * @param x 输入信号(待滤波信号)
   * @param sampTime 采样周期
   */
  Vector filt(Vector& x, double sampTime) {
    // 一阶低通滤波器的离散化实现
    // y[n] = k1*y[n-1] + k2*(x[n]+x[n-1])
    // y1=y[n-1]
    // x=x[n], x1=x[n-1]
    y1 = k1 * y1 + k2 * (x + x1);
    x1 = x;
    return y1;
  }

  /**
   * @brief 更新滤波器参数
   * @param cutOff 截至频率
   * @param sampTime 采样时间
   */
  void update(double cutOff, double sampTime) {
    // coeffi = omega_a/(2*fs) = tan(omega_d * Ts /2)
    coeffi = tan(cutOff * sampTime / 2);
    k1 = (1 - coeffi) / (1 + coeffi);
    k2 = coeffi / (1 + coeffi);
  }

  /**
   * @brief 初始化输入输出
   */
  void set(Vector& x0) {
    x1 = x0;
    y1 = coeffi / (1 + coeffi) * x0;
  }

 private:
  Vector x1, y1;
  double k1, k2, coeffi;
};

////////////////////////////////////////////////////////////////
// H(s)=-w^2/(s+w)
class FilterF2 : public FilterIIR {
 public:
  FilterF2(double cutOff, double sampTime, int N) : FilterIIR(), x1(Vector(N)), y1(Vector(N)) {
    update(cutOff, sampTime);
  }

  /**
   * @brief 滤波
   * @param x 输入信号(待滤波信号)
   * @param sampTime 采样周期
   */
  Vector filt(Vector& x, double sampTime) {
    // y[n] = k1*y[n-1] + k2*(x[n] + x[n-1])
    // y1 = y[n-1]
    // x1 = x[n-1], x = x[n]
    y1 = k1 * y1 + k2 * (x + x1);
    x1 = x;
    return y1;
  }

  /**
   * @brief 更新滤波器参数
   * @param cutOff 截至频率
   * @param sampTime 采样时间
   */
  void update(double cutOff, double sampTime) {
    cut = cutOff;
    f2 = 2 / sampTime;                    // f2 = 2*fs
    coeffi = tan(cutOff * sampTime / 2);  // coeffi = omega_a/(2*fs) = tan(omega_d * Ts /2)
    k1 = (1 - coeffi) / (1 + coeffi);
    k2 = -f2 * coeffi * coeffi / (1 + coeffi);
  }

  /**
   * @brief 初始化输入输出
   */
  void set(Vector& x0) {
    x1 = x0;
    y1 = -f2 * coeffi * x0;
    // y1 = 0*x0;
  }

  /**
   * @brief 返回截止角频率
   */
  double getOmega() {
    return cut;
  }

 private:
  Vector x1, y1;
  double f2, coeffi, k1, k2, cut;
};

#endif