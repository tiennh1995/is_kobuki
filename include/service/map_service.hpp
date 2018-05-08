#ifndef MAP_SERVICE_HPP
#define MAP_SERVICE_HPP

#include <cstdlib>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <vector>
#include <iostream>
#include <unistd.h>
#include <is_kobuki/UpdateMap.h>
#include <is_kobuki/ValidMegaCells.h>
#include <is_kobuki/UpdateRobot.h>
#include <is_kobuki/DivideMap.h>
#include <ecl/threads.hpp>
#include "common/common.hpp"
#include "common/robot_status.hpp"
#include <time.h>

class MapService {
private:
  ros::NodeHandle nh;

  // Subscribers
  ros::Subscriber map_sub;
  ros::Subscriber map_metadata_sub;

  // Services
  ros::ServiceServer update_map_service;
  ros::ServiceServer valid_mega_cells_service;
  ros::ServiceServer divide_map_service;

  // Map
  Map map;
  bool initMap;

  // Robot
  int robotSize;
  Robot* robots;
  int MAX_ROBOT_SIZE;
  int cellSize;
  double duration1, duration2;
  double timeDied;


  void initializeMap(const nav_msgs::MapMetaDataConstPtr msg);
  void setMapData(const nav_msgs::OccupancyGridConstPtr msg);
  void initCell();

  bool updateMap(is_kobuki::UpdateMap::Request& request,
                 is_kobuki::UpdateMap::Response& response);
  bool validMegaCells(is_kobuki::ValidMegaCells::Request& request,
                      is_kobuki::ValidMegaCells::Response& response);
  bool divideMap(is_kobuki::DivideMap::Request& request,
                   is_kobuki::DivideMap::Response& response);
  int findRobot(int robotId);
  bool validRobotId(int robotId);
  bool validCell(int row, int col);

public:
  MapService(ros::NodeHandle nodehandle) {
    nh = nodehandle;
    map = Map();
    initMap = true;
    robotSize = 0;
    MAX_ROBOT_SIZE = 2;
    robots = (Robot*)malloc(MAX_ROBOT_SIZE * sizeof(Robot));
    cellSize = 16;
    timeDied = 5;
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
