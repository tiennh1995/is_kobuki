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

#include "../stc/stc_navigation.hpp"

class KobukiController {
private:
  // ROS handle
  ros::NodeHandle nh;

  // Subscribers
  ros::Subscriber enable_controller_sub;
  ros::Subscriber disable_controller_sub;
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

  geometry_msgs::Twist twist;
  move_base_msgs::MoveBaseGoal move_base_goal;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_movebase_client;

  // STC controller
  STCNavigation stcNavigation;

  // Map
  Map map;

  // Custom variable

  // Use when moveTo
  int maxRetry;
  int retry;
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
  float angularSpeed;
  float linearSpeed;

  ecl::Thread thread;
  ecl::Thread thread2;

public:
  KobukiController(ros::NodeHandle nodehandle, std::string movebase_ns): action_movebase_client(nodehandle, movebase_ns, true) {
    nh = nodehandle;
    stcNavigation = STCNavigation();
    map = Map();
    maxRetry = 10;
    retry = 0;
    initMap = true;
    up = 0;
    left = 0;
    down = 0;
    right = 0;
    cellSize = 0;
    currentCellX = 0;
    currentCellY = 0;
    maxAMCL = 0;
    PI = 3.1415926535897;
    angularSpeed = PI / 8;
    linearSpeed = 0.125;
  }

  // Destructor
  ~KobukiController() {
    if (!map.isNULL()) {
      free(map.getMapData());
    }
  }

  // Declare methods
  bool init();

  // Khoi tao map
  void initializeMap(const nav_msgs::MapMetaDataConstPtr msg);
  void setMapData(const nav_msgs::OccupancyGridConstPtr msg);

  // Khoi tao cell, megaCell
  void initCell();

  // Chay robot voi thuat toan STC
  void moveWithSTC();

  // Di chuyen giua cac megaCell (Tu currentMegaCell toi megaCell)
  void moveToMegaCell(MegaCell megaCell);

  // Khoi tao move_base
  bool initMoveBase();

  // Di chuyen giua cac cell (Tu currentCell toi nextCell)
  bool moveToCell(Cell nextCell, int direction);

  void turn(int degree);
  void turnLeft();
  void turnRight();
  void go(float distance);
  void goForward(float distance);
  void goStraight(float distance);

  // Xu li cac su kien tuong ung
  void bumperEventHandle(const kobuki_msgs::BumperEventConstPtr msg);
  void bumperEventHandle(const kobuki_msgs::CliffEventConstPtr msg);
  void odometryHandle(const nav_msgs::OdometryConstPtr odometry);
  void laserHandle(const sensor_msgs::LaserScanConstPtr laser);
  void amclHandle(const geometry_msgs::PoseWithCovarianceStampedConstPtr amclpose);
};

#endif
