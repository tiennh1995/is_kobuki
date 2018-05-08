#ifndef KOBUKI_CONTROLLER_H
#define KOBUKI_CONTROLLER_H

#include <cmath>
#include <cstdlib>
#include <ctime>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/CliffEvent.h>
#include <kobuki_msgs/MotorPower.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <ros/ros.h>
#include <ecl/threads.hpp>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <time.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <unistd.h>
#include <string>
#include <tf/tf.h>
#include <geometry_msgs/Pose2D.h>

#include "../stc/stc_navigation.hpp"
#include "../service/map_service.hpp"

class KobukiController {
private:
  // ROS handle
  ros::NodeHandle nh;

  // Subscribers
  ros::Subscriber bumper_event_sub;
  ros::Subscriber cliff_event_sub;
  ros::Subscriber map_sub;
  ros::Subscriber map_metadata_sub;
  ros::Subscriber amcl_sub;
  ros::Subscriber odometry_sub;
  ros::Subscriber laser_sub;

  // Publishers
  ros::Publisher cmd_vel_pub;
  ros::Publisher power_pub;
  ros::Publisher amcl_pub;

  // Service
  ros::ServiceClient updateMapClient;
  ros::ServiceClient validMegaCellsClient;
  ros::ServiceClient divideMapClient;

  // STC controller
  STCNavigation stcNavigation;

  // Message
  geometry_msgs::Twist twist;
  geometry_msgs::Pose2D current_pose;

  // Map
  Map map;

  // Custom variable
  int robotId;

  // Use when moveTo
  bool initMap;
  int up;
  int left;
  int down;
  int right;
  int cellSize;
  int currentCellX;
  int currentCellY;
  int maxAMCL;

  double PI;
  double PI_MOVE;
  float angularSpeed;
  float linearSpeed;
  bool isDirectionX;
  float deltaX, deltaY, deltaTheta;
  float epsilonX, epsilonY, epsilonTheta;

  ecl::Thread thread1;

  int minRow;
  int maxRow;
  int minCol;
  int maxCol;

  std::vector<int> rows;
  std::vector<int> cols;
  std::vector<Cell> cellArr;
  Cell certainCell;

public:
  KobukiController(ros::NodeHandle nodehandle) {
    nh = nodehandle;
    stcNavigation = STCNavigation();
    map = Map();
    initMap = true;
    cellSize = 16;
    PI = 3.1415926535897;
    PI_MOVE = 3;
    angularSpeed = PI / 8;
    linearSpeed = 0.125;
    isDirectionX = true;
    epsilonX = 0.05;
    epsilonY = 0.05;
    epsilonTheta = 0.05;
    robotId = 0;
    minRow = 0;
    minCol = 0;
    maxRow = 0;
    maxCol = 0;
  }

  // Destructor
  ~KobukiController() {
    if (!map.isNULL()) {
      free(map.getMapData());
    }
  }

  int getRobotId();
  void setRobotId(int robotId);

  bool getIsDirectionX();
  void setIsDirectionX(bool isDirectionX);

  // Declare methods
  bool init();

  std::string initTopic(std::string topic);

  // Khoi tao map
  void initializeMap(const nav_msgs::MapMetaDataConstPtr msg);
  void setMapData(const nav_msgs::OccupancyGridConstPtr msg);

  // Khoi tao cell, megaCell
  void initCell();

  // Chay robot voi thuat toan STC
  void moveWithSTC();

  // Di chuyen giua cac megaCell (Tu currentMegaCell toi megaCell)
  void moveToMegaCell(MegaCell megaCell);

  // Di chuyen giua cac cell (Tu currentCell toi nextCell)
  bool moveToCell(Cell nextCell, int direction);

  void turn(float degree);
  void turnLeft();
  void turnRight();
  void go(float distance);
  void goForward(float distance);
  void goStraight(float distance);
  void reMove();

  // Mapservice
  int updateMap(Cell cell, int isFinish);
  bool validMegaCells(int robotId);
  void divideMap(int robotId);

  // Xu li cac su kien tuong ung
  void bumperEventHandle(const kobuki_msgs::BumperEventConstPtr msg);
  void cliffEventHandle(const kobuki_msgs::CliffEventConstPtr msg);
  void odometryHandle(const nav_msgs::OdometryConstPtr& odometry);
  void laserHandle(const sensor_msgs::LaserScanConstPtr laser);
  void amclHandle(const geometry_msgs::PoseWithCovarianceStampedConstPtr amclpose);
  void printCellAndMegaCell();
  void goToCertainCell(Cell **cell, Cell beginCell, Cell certainCell);
};

#endif
