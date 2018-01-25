#include "../include/controller/kobuki_controller.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "is_kobuki_node");
  std::string movebase_ns = std::string("move_base");
  ros::NodeHandle nh;
  KobukiController kobukiController(nh, movebase_ns);
  kobukiController.init();

  // ros::Rate loop_rate(10);
  // while (ros::ok()) {
  //   loop_rate.sleep();
  // }

  return 0;
}
