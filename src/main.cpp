#include "../include/controller/kobuki_controller.hpp"

int main(int argc, char **argv) {
  std::string nodeName = "is_kobuki_node";
  if (argc > 1)
    nodeName += argv[1];

  ros::init(argc, argv, nodeName);
  ros::NodeHandle nh;
  KobukiController kobukiController(nh);
  if (argc > 1)
    kobukiController.setRobotId(atoi(argv[1]));
  kobukiController.init();
  return 0;
}
