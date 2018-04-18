#ifndef MAP_SERVICE_HPP
#define MAP_SERVICE_HPP

#include <cstdlib>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <vector>
#include <is_kobuki/UpdateMap.h>
#include <is_kobuki/ValidMegaCells.h>
#include <is_kobuki/UpdateRobot.h>
#include "common/common.hpp"

class MapService {
private:
  ros::NodeHandle nh;

  // Subscribers
  ros::Subscriber map_sub;
  ros::Subscriber map_metadata_sub;

  // Services
  ros::ServiceServer update_robot_service;
  ros::ServiceServer update_map_service;
  ros::ServiceServer valid_mega_cells_service;

  // Map
  Map map;
  bool initMap;

  // Robot
  int robotSize;
  Robot* robots;
  int cellSize;

  void initializeMap(const nav_msgs::MapMetaDataConstPtr msg);
  void setMapData(const nav_msgs::OccupancyGridConstPtr msg);
  void initCell();

  bool robotJoin(is_kobuki::UpdateRobot::Request& request, is_kobuki::UpdateRobot::Response& response);
  bool updateMap(is_kobuki::UpdateMap::Request& request, is_kobuki::UpdateMap::Response& response);
  bool validMegaCells(is_kobuki::ValidMegaCells::Request& request, is_kobuki::ValidMegaCells::Response& response);
  bool updateRobot(is_kobuki::UpdateRobot::Request& request, is_kobuki::UpdateRobot::Response& response);

public:
  MapService(ros::NodeHandle nodehandle) {
    nh = nodehandle;
    map = Map();
    initMap = true;
    robotSize = 0;
    robots = (Robot*)malloc(2 * sizeof(Robot));
    cellSize = 6;
  }

  ~MapService() {
    if (!map.isNULL()) {
      free(map.getMapData());
    }

    free(robots);
  }

  bool init();
};

#endif
