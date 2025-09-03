#include "ImpedanceControl/impedanceControlExpFakeWrench.h"
#include "Log/logger.h"

int main(int argc, char* argv[]) {
  if (argc != 7) {
    LOG(ERROR, "There must be 6 parameter of the mask.");
    exit(-1);
  }
  const int observeType = 4;
  const std::string ip = "192.168.0.2";
  std::vector<double> Ma = {5, 5, 5, 1, 1, 1};
  std::vector<double> Da = {6, 6, 6, 1, 1, 1};
  std::vector<double> Ka = {10, 10, 10, 1, 1, 1};
  const std::string paramType = "normal";
  std::shared_ptr<UR5> ur5 = std::make_shared<UR5>();
  ImpedanceControl ic(Ma, Da, Ka, 50, observeType, ip, paramType, ur5);
  const std::vector<int> mask = {atoi(argv[1]), atoi(argv[2]), atoi(argv[3]), atoi(argv[4]), atoi(argv[5]), atoi(argv[6])};
  ic.runImpedanceController(mask);
  return 0;
}