#include "../include/service/map_service.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "is_kobuki_map_node");
  ros::NodeHandle nh;
  MapService mapService(nh);
  mapService.init();
  return 0;
}
