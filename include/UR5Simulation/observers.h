#ifndef OBSERVERS_H
#define OBSERVERS_H
#include <memory>
#include <string>

#include "ExtObserverLib_v3/external_observer.h"
std::shared_ptr<ExternalObserverRnea> getObserver(RobotDynamicsRnea* robot, const int& observerType, const double& timeStep, const std::string&);

#endif