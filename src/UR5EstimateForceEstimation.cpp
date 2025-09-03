#include <boost/program_options.hpp>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <string>

#include "UR5Simulation/UR5SimulationModel.h"
#include "UR5Simulation/observers.h"
#include "gnuplot-iostream.h"

using namespace std;

// 高斯噪声设置
#define MEAN 0.0     // 均值
#define STDDEV 0.01  // 标准差

Vector getConstTauExt() {
  Vector tmp;
  tmp.resize(6);
  tmp << 5, 5, 5, 2, 2, 2;
  return tmp;
}

Vector getVaryTauExt(int i, int startTime, int stopTime) {
  double base1 = sin(M_PI * 2 * (i - startTime) / (stopTime - startTime));
  double base2 = sin(M_PI * 6 * (i - startTime) / (stopTime - startTime));
  Vector tmp;
  tmp.resize(6);
  tmp << 4 * base1 + base2, 4 * base1 + base2, 4 * base1 + base2, 2 * base1 + base2, 2 * base1 + base2, 2 * base1 + base2;
  return tmp;
}

int main(int argc, char **argv) {
  // 设置参数
  boost::program_options::options_description description("Allow options");
  description.add_options()("help,h", "Turn the gain parameter for external observer.")("externalTorqueType,e", boost::program_options::value<string>(), "external torque type: const or vary")("parameterType,p", boost::program_options::value<string>()->default_value("normal"), "parameter type: normal, small or big.")("inputFile,i", boost::program_options::value<string>(), "input file for external observer.")("outputFile,o", boost::program_options::value<string>(), "output file for saving data.")("frequency,f", boost::program_options::value<double>()->default_value(100), "sample Frequency")("observerType,t",
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    boost::program_options::value<int>(),
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    "observer type:\n \
    0: momentum observer\n \
    1: nonlinear observer\n \
    2: sliding mode observer\n \
    3: filtered dynamic observer\n \
    4: kalman filter observer(Tayler)\n \
    5: kalman filter observer(Zero order filter)\n");

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, description), vm);
  boost::program_options::notify(vm);
  if (vm.count("help")) {
    std::cout << description << std::endl;
    return 0;
  }
  const string externalTorqueType = vm["externalTorqueType"].as<string>();
  const string parameterType = vm["parameterType"].as<string>();
  const string inputFilePrefix = std::filesystem::path(__FILE__).parent_path().parent_path().string() + "/Bullet2.83/";
  string inputFile = inputFilePrefix + vm["inputFile"].as<string>();
  const string outputFilePrefix = filesystem::path(__FILE__).parent_path().parent_path().string() + "/output/";
  string outputFile = outputFilePrefix + vm["outputFile"].as<string>();
  const double timeStep = 1 / vm["frequency"].as<double>();
  const int observerType = vm["observerType"].as<int>();

  // 机器人实例
  UR5 ur5Robot;
  std::shared_ptr<ExternalObserverRnea> observer = getObserver(&ur5Robot, observerType, timeStep, parameterType);

  // 高斯噪声
  default_random_engine generator;
  normal_distribution<double> dist(MEAN, STDDEV);
  // 加载数据
  const int dof = ur5Robot.jointNo();
  Vector q(dof), qd(dof), tau(dof), tauExtMeasured(dof), tauExtEstimated(dof);
  q.setZero();
  qd.setZero();
  tau.setZero();
  tauExtMeasured.setZero();
  tauExtEstimated.setZero();
  // 画图
  vector<vector<tuple<double, double>>> velPlot(dof);
  vector<vector<tuple<double, double>>> tauExtMeasuredPlot(dof), tauExtEstimatedPlot(dof);

  ofstream ofs(outputFile);
  ofs << "t, qd1, qd2, qd3, qd4, qd5, qd6,"
      << "tauMea1, tauEst1, tauMea2, tauEst2, tauMea3, tauEst3, tauMea4, tauEst4, tauMea5, tauEst5, tauMea6, tauEst6" << endl;
  ifstream ifs(inputFile);
  string line;
  getline(ifs, line);
  int row = 0;
  int extStart = 800;
  int extStop = 1200;
  while (getline(ifs, line)) {
    int cnt = 0;
    stringstream ss(line);
    string word;
    while (getline(ss, word, ',')) {
      double tmp = atof(word.c_str());
      if (cnt < 6) {
        q(cnt % dof) = tmp;
      } else if (cnt < 12) {
        qd(cnt % dof) = tmp + dist(generator);
      } else if (cnt >= 24 && cnt < 30) {
        tau(cnt % dof) = tmp;
      }
      ++cnt;
    }
    tauExtMeasured.setZero();
    // 在[extStart, extStop]时间内人为添加外力
    if (row > extStart && row < extStop) {
      if (externalTorqueType == "const") {
        tauExtMeasured = getConstTauExt();
      } else {
        tauExtMeasured = getVaryTauExt(row, extStart, extStop);
      }
      for (unsigned int index = 0; index < tau.size(); ++index) {
        tau(index) -= tauExtMeasured(index);
      }
    }
    double t = row * timeStep;
    ++row;
    tauExtEstimated = observer->getExternalTorque(q, qd, tau, timeStep);
    // 保存数据
    ofs << t;
    for (unsigned int i = 0; i < dof; ++i) {
      ofs << "," << qd(i);
    }
    for (unsigned int i = 0; i < dof; ++i) {
      ofs << "," << tauExtMeasured(i) << "," << tauExtEstimated(i);
    }
    ofs << endl;
    // 画图数据
    for (unsigned int index = 0; index < dof; ++index) {
      velPlot[index].push_back(make_tuple(t, qd(index)));
      tauExtEstimatedPlot[index].push_back(make_tuple(t, tauExtEstimated(index)));
      tauExtMeasuredPlot[index].push_back(make_tuple(t, tauExtMeasured(index)));
    }
  }
  ifs.close();
  ofs.close();

  // 调用gnuplot画图
  Gnuplot gp;
  gp << "set term qt 1 font \",20\" size 1920, 1080\n";
  // gp << "set output \"" + outputFile +"\"\n";
  gp << "set multiplot layout 3,2\n";
  for (unsigned int index = 0; index < dof; ++index) {
    gp << "set ytics nomirror\nset y2tics\n";
    gp << "set xlabel \"time\"\nset ylabel \"torque\"\nset y2label \"velocity\"\n";
    gp << "plot '-' with linespoints linecolor " << to_string(index + 1) << " linewidth 0.6 pointtype " << to_string(index + 1) << " pointsize 1.0 title \"measured_" << to_string(index + 1) << "\" axis x1y1,"
       << "'-' with linespoints linecolor " << to_string(index + 2) << " linewidth 0.6 pointtype " << to_string(index + 2) << " pointsize 1.0 title \"estimated_" << to_string(index + 1) << "\" axis x1y1,"
       << "'-' with linespoints linecolor " << to_string(index + 3) << " linewidth 0.6 pointtype " << to_string(index + 3) << " pointsize 1.0 title \"vel_" << to_string(index + 1) << "\" axis x1y2\n";
    gp.send1d(tauExtMeasuredPlot[index]);
    gp.send1d(tauExtEstimatedPlot[index]);
    gp.send1d(velPlot[index]);
  }
  gp << "unset multiplot\n";
  return 0;
}