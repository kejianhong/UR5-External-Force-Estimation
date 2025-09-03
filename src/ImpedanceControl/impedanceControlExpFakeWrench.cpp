#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>

using namespace std::chrono;

#include "ImpedanceControl/impedanceControlExpFakeWrench.h"
#include "Log/logger.h"
#include "UR5Simulation/observers.h"

bool ImpedanceControl::runtimeFlag = true;

ImpedanceControl::ImpedanceControl(std::vector<double>& Ma, std::vector<double>& Da, std::vector<double>& Ka, int fre, int oType, const std::string& ip, const std::string& parameterType, std::shared_ptr<RobotDynamicsRnea> rd, double maxAcc, double maxVel, double acc, double vel, double lT, double gain) : initM(Eigen::Map<const Vector>(Ma.data(), Ma.size())), initD(Eigen::Map<const Vector>(Ka.data(), Ka.size())), initK(Eigen::Map<const Vector>(Ka.data(), Ka.size())), M(Eigen::Map<Vector>(Ma.data(), Ma.size())), D(Eigen::Map<Vector>(Da.data(), Da.size())), K(Eigen::Map<Vector>(Ka.data(), Ka.size())), parameterType(parameterType), frequency(fre), dt(1.0 / frequency), observerType(oType), rtde_control(ip), rtde_receive(ip), robot(rd), maxVel(maxVel), maxAcc(maxAcc), acceleration(acc), velocity(vel), lookaheadTime(lT), gain(gain), tcpTauExt(Vector::Zero(6)), error(Vector::Zero(6)), derror(Vector::Zero(6)), dderror(Vector::Zero(6)) {
  init();
  observer = getObserver(robot.get(), observerType, dt, parameterType);
  const auto equilibriumPose = getTcpTransformPose(rtde_receive.getActualTCPPose());
  currTranslation = equilibriumTranslation = std::get<0>(equilibriumPose);
  currQuaternion = equilibriumQuaternion = std::get<1>(equilibriumPose);
}

ImpedanceControl::~ImpedanceControl() {
  rtde_control.servoStop();
  rtde_control.stopScript();
}

void ImpedanceControl::computeImpedance() {
  dderror = M.asDiagonal().inverse() * (tcpTauExt - D.asDiagonal() * derror - K.asDiagonal() * error);
  derror += dt * dderror;
  auto currTCPPose = getTcpTransformPose(rtde_receive.getActualTCPPose());
  currTranslation = std::get<0>(currTCPPose);
  currQuaternion = std::get<1>(currTCPPose);
  // Shortest arc.
  if (equilibriumQuaternion.coeffs().dot(currQuaternion.coeffs()) < 1e-3) {
    currQuaternion.coeffs() = -currQuaternion.coeffs();
  }
  // Normalize the quaternion error.
  clearLog();
  Eigen::Quaterniond quaternionError(currQuaternion * equilibriumQuaternion.inverse());
  const double quaternionDiffNorm = (Eigen::Quaterniond::Identity().coeffs() - quaternionError.coeffs()).norm();
  if (quaternionDiffNorm > 1e-5) {
    quaternionError.normalize();
  } else {
    quaternionError = Eigen::Quaterniond::Identity();
  }
  ss << "quaternionError=[" << quaternionError.coeffs().transpose() << "], ";
  // Update the error vector.
  error.topRows(3) = currTranslation - equilibriumTranslation;
  ss << "translationError =[" << error.topRows(3).transpose() << "], ";
  Eigen::AngleAxisd angleAixsError(quaternionError);
  error.bottomRows(3) << angleAixsError.angle() * angleAixsError.axis();
  ss << "angleAxisError =[" << error.bottomRows(3).transpose() << "]";
  LOG(INFO, ss.str());
}

void ImpedanceControl::limitVelocity() {
  const double epsilon = 1e-6;
  double tcpVelNorm = derror.topRows(3).norm();
  if (tcpVelNorm > maxVel) {
    clearLog();
    ss << " Impedance control generate fast movements! Velocity norm = " << tcpVelNorm;
    LOG(WARNING, ss.str());
    derror.segment(0, 3) *= (maxVel / tcpVelNorm);
  }
  if (tcpVelNorm < epsilon) {
    derror.segment(0, 3).setZero();
  }
  double tcpOmegaX = std::fabs(derror(3));
  double tcpOmegaY = std::fabs(derror(4));
  double tcpOmegaZ = std::fabs(derror(5));
  if (tcpOmegaX > maxVel) {
    clearLog();
    ss << "Velocity along x is too fast! Vel x = " << derror(3);
    LOG(WARNING, ss.str());
    derror(3) *= (maxVel / tcpOmegaX);
  }
  if (tcpOmegaY > maxVel) {
    clearLog();
    ss << "Velocity along y is too fast! Vel y = " << derror(4);
    LOG(WARNING, ss.str());
    derror(4) *= (maxVel / tcpOmegaY);
  }
  if (tcpOmegaZ > maxVel) {
    clearLog();
    ss << "Velocity along z is too fast! Vel z = " << derror(5);
    LOG(WARNING, ss.str());
    derror(5) *= (maxVel / tcpOmegaZ);
  }
  if (tcpOmegaX < epsilon) {
    derror(3) = 0;
  }
  if (tcpOmegaY < epsilon) {
    derror(4) = 0;
  }
  if (tcpOmegaZ < epsilon) {
    derror(5) = 0;
  }
}

std::tuple<Vector, Eigen::Quaterniond> ImpedanceControl::getTcpTransformPose(const std::vector<double>& currTCPPose) {
  const Vector pose = Eigen::Map<const Vector>(currTCPPose.data(), currTCPPose.size());
  Vector translation(pose.topRows(3));
  Vector angleAxis(pose.bottomRows(3));
  Eigen::Quaterniond quaternion;
  if (angleAxis.norm() < 1e-6) {
    quaternion = Eigen::Quaterniond::Identity();
  } else {
    Eigen::AngleAxisd angleAxisNorm(angleAxis.norm(), angleAxis / angleAxis.norm());
    quaternion = Eigen::Quaterniond(angleAxisNorm);
  }
  return std::make_tuple(translation, quaternion);
}

void ImpedanceControl::init() {
  const std::vector<double> qInit = {-M_PI / 2, -M_PI / 2, -M_PI / 3 * 2, -M_PI / 4, M_PI / 2, 0};
  rtde_control.moveJ(qInit);
  sleep(1);
}

void ImpedanceControl::clearLog() {
  ss.str("");
  ss.clear();
}

void ImpedanceControl::runImpedanceController(const std::vector<int>& mask) {
  LOG(WARNING, "Start impedance control.");
  // Press [ctrl+c] to stop the program.
  signal(SIGINT, signalCallbackFunc);

  const double forceThreshold = 20;
  const double torqueThreshold = 5;
  const double resultantForceThreshold = 40;
  int continueForceCount = 0;
  const int continueForceCountThreshold = 5;  // The force should exceed the force-torque threshold more than this parameter.
  const int externalForceNum = 3000;

  // Fake the external force-torque applied to the TCP.
  Matrix appliedFakeExternalForce(6, externalForceNum);  // Each row represents the applied external force-torque on the TCP.
  appliedFakeExternalForce.setZero();
  const int offset = 500;
  for (int i = 50; i <= 150; ++i) {
    // External force and torque are not applied at the same time.
    appliedFakeExternalForce(0, i) = appliedFakeExternalForce(1, i) = appliedFakeExternalForce(2, i) = 30;                             // External force.
    appliedFakeExternalForce(3, i + offset) = appliedFakeExternalForce(4, i + offset) = appliedFakeExternalForce(5, i + offset) = 15;  // External torque.
  }

  int forceIndex = 0;
  while (runtimeFlag && forceIndex++ < externalForceNum - 1) {
    std::vector<double> q = rtde_receive.getActualQ();
    std::vector<double> qd = rtde_receive.getActualQd();
    std::vector<double> tau = rtde_receive.getActualCurrent();
    for (size_t i = 0; i < tau.size(); ++i) {
      // Convert the joint current to the joint torque.
      tau[i] *= motorGain[i];
    }
    Vector qVector = Eigen::Map<Vector>(q.data(), q.size());
    Vector qdVector = Eigen::Map<Vector>(qd.data(), qd.size());
    Vector tauVector = Eigen::Map<Vector>(tau.data(), tau.size());
    tcpTauExt = robot->getVelocityJacobianBase(qVector).transpose().inverse() * observer->getExternalTorque(qVector, qdVector, tauVector, dt);
    // Fake the estimated external force-torque.
    tcpTauExt = appliedFakeExternalForce.col(forceIndex);
    for (size_t mi = 0; mi < mask.size(); ++mi) {
      // Setting the mask to filter the external force-torque.
      tcpTauExt(mi) *= mask[mi];
    }

    double externalForceNorm = tcpTauExt.segment(0, 3).norm();
    double Fx = std::fabs(tcpTauExt(0));
    double Fy = std::fabs(tcpTauExt(1));
    double Fz = std::fabs(tcpTauExt(2));
    double Mx = std::fabs(tcpTauExt(3));
    double My = std::fabs(tcpTauExt(4));
    double Mz = std::fabs(tcpTauExt(5));
    if (externalForceNorm > resultantForceThreshold) {
      continueForceCount++;
    } else {
      continueForceCount = 0;
    }

    clearLog();
    ss << "forceIndex=" << forceIndex << "/" << externalForceNum << ", state=" << state << ", continueForceCount=" << continueForceCount << ", [Fx, Fy, Fz, Mx, My, Mz]=[" << tcpTauExt.transpose();
    ss << "], externalForceNorm=" << externalForceNorm << ", jointTorque=[";
    for (const double& i : tau) {
      ss << i << ", ";
    }
    ss << "]";
    LOG(INFO, ss.str());

    const bool isAppliedExternalForce = (continueForceCount > continueForceCountThreshold) && (Fx > forceThreshold || Fy > forceThreshold || Fz > forceThreshold);
    if (isAppliedExternalForce) {
      state = 1;
    } else {
      tcpTauExt.topRows(3).setZero();
    }
    if (Mx > torqueThreshold) {
      state = 1;
    } else {
      tcpTauExt(3) = 0;
    }
    if (My > torqueThreshold) {
      state = 1;
    } else {
      tcpTauExt(4) = 0;
    }
    if (Mz > torqueThreshold) {
      state = 1;
    } else {
      tcpTauExt(5) = 0;
    }

    // When all the external force-torque is zero, reset the state.
    if (tcpTauExt.norm() < 1e-3) {
      state = 0;
    }
    // Update or reset the state based on the state.
    clearLog();
    if (state == 0) {
      updateParamMatrix();
    } else {
      LOG(INFO, "State=1, applying the external force, reset the parameter.");
      resetParamMatrix();
    }

    computeImpedance();
    limitVelocity();
    q = rtde_receive.getActualQ();                                          // Update the current joint value.
    qdVector = robot->getVelocityJacobianBase(qVector).inverse() * derror;  // Resue the `qdvector` to store the joint velocity.
    if (qVector.norm() <= 1e-5) {
      qdVector.setZero();
    }
    clearLog();
    ss << "Tcp velocity increment=[";
    for (size_t qIndex = 0; qIndex < q.size(); ++qIndex) {
      ss << derror(qIndex) << ", ";
      q[qIndex] += qdVector(qIndex) * dt;
    }
    ss << "]";
    LOG(INFO, ss.str());

    // Start to move.
    auto timeStart = high_resolution_clock::now();
    rtde_control.servoJ(q, velocity, acceleration, dt, lookaheadTime, gain);
    auto timeStop = high_resolution_clock::now();
    auto timeDuration = duration<double>(timeStop - timeStart);
    if (timeDuration.count() < dt) {
      std::this_thread::sleep_for(duration<double>(dt - timeDuration.count()));
    }
  }
  rtde_control.servoStop();
  rtde_control.stopScript();
  LOG(WARNING, "Finished impedance control.");
}

void ImpedanceControl::resetParamMatrix() {
  for (unsigned int i = 0; i < 6; ++i) {
    M(i) = initM(i);
    D(i) = initD(i);
    K(i) = initK(i);
  }
}

void ImpedanceControl::updateParamMatrix() {
  for (unsigned int i = 0; i < 6; ++i) {
    const double scale = std::exp(std::fabs(derror(i)));
    M(i) = initM(i) / scale;
    K(i) = 1.5 * initK(i) * scale;
  }
}