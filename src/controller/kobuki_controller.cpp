#include "../../include/controller/kobuki_controller.hpp"

bool KobukiController::init() {
  // Subscriber
  // Init map
  map_metadata_sub = nh.subscribe("/map_metadata", 10,
                                  &KobukiController::initializeMap, this);
  // bumper_event_sub = nh.subscribe("/mobile_base/events/bumper",
  //                                  1, &KobukiController::bumperEventHandle, this);
  // cliff_event_sub = nh.subscribe("/mobile_base/events/cliff", 1,
  //                                &KobukiController::bumperEventHandle, this);
  // odometry_sub = nh.subscribe("/odom", 1,
  //                             &KobukiController::odometryHandle, this);
  // laser_sub = nh.subscribe("/scan", 1,
  //                          &KobukiController::laserHandle, this);
  amcl_sub = nh.subscribe("/amcl_pose", 1,
                          &KobukiController::amclHandle, this);

  // Publisher
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  power_pub = nh.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 1);
  amcl_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

  thread.start(&KobukiController::moveWithSTC, *this);

  ros::spin();

  return true;
}

// Init map
void KobukiController::initializeMap(const nav_msgs::MapMetaDataConstPtr msg) {
  if (initMap) {
    map.setHeight(msg->height);
    map.setWidth(msg->width);
    map.setResolution(msg->resolution);
    map.setOriginX(msg->origin.position.x);
    map.setOriginY(msg->origin.position.y);

    map_sub = nh.subscribe("/map", 10,
                           &KobukiController::setMapData, this);
    initMap = false;
  }
}

void KobukiController::setMapData(const nav_msgs::OccupancyGridConstPtr msg) {
  std::vector<int> mapData;

  for (int i = 0; i < msg->data.size(); i++) {
    mapData.push_back((int) msg->data[i]);
  }

  map.setMapData(map.oneArrToTwoArr(mapData));
  initCell();
}

// Khoi tao cell, megaCell
void KobukiController::initCell() {
  up = map.findLineUp();
  left = map.findLineLeft();
  down = map.findLineDown();
  right = map.findLineRight();

  // Fix cung
  cellSize = 6;

  // khoi tao mang cells

  // Tinh so dong sau khi thu nho
  int numberRow = ((up - down + 1) / cellSize);
  if (numberRow % 2 != 0)
    numberRow--;

  // Tinh so cot sau khi thu nho
  int numberCol = (right - left + 1) / cellSize;
  if (numberCol % 2 != 0)
    numberCol--;

  // Cat bo phan pixel thua sao cho ban do vua khop so Cell
  left = right - numberCol * cellSize + 1;

  down = up - numberRow * cellSize + 1;

  Common::cells = new Cell*[numberRow];
  for (int i = 0; i < numberRow; i++)
    Common::cells[i] = new Cell[numberCol];

  int col = left;
  int row = down;
  for (int rowC = 0; rowC < numberRow; rowC++) {
    for (int colC = 0; colC < numberCol; colC++) {
      Common::cells[rowC][colC].setX(col);
      Common::cells[rowC][colC].setY(row + cellSize - 1);
      Common::cells[rowC][colC].setCentreX(col + cellSize / 2);
      Common::cells[rowC][colC].setCentreY(row + cellSize - 1 - cellSize / 2);
      Common::cells[rowC][colC].setCellSize(cellSize);
      Common::cells[rowC][colC].setObstacle(map.checkObstacle(col, row + cellSize - 1, cellSize));
      col = col + cellSize;
    }
    col = left;
    row = row + cellSize;
  }
  Common::rowCells = numberRow;
  Common::colCells = numberCol;

  // Cap phat dong cho mang megaCells
  Common::megaCells = new MegaCell*[numberRow / 2];
  for (int i = 0; i < numberRow / 2; i++)
    Common::megaCells[i] = new MegaCell[numberCol / 2];

  // tao mang megaCells
  int rowM = 0;
  int colM = 0;
  bool obstacle = false;

  for (int rowC = 0; rowC < numberRow; rowC = rowC + 2) {
    for (int colC = 0; colC < numberCol; colC = colC + 2) {
      if (Common::cells[rowC][colC].hasObstacle()) obstacle = true;
      else if (Common::cells[rowC + 1][colC].hasObstacle()) obstacle = true;
      else if (Common::cells[rowC][colC + 1].hasObstacle()) obstacle = true;
      else if (Common::cells[rowC + 1][colC + 1].hasObstacle()) obstacle = true;
      else obstacle = false;

      Common::megaCells[rowM][colM].setX(Common::cells[rowC + 1][colC].getX());
      Common::megaCells[rowM][colM].setY(Common::cells[rowC + 1][colC].getY());
      Common::megaCells[rowM][colM].setCellSize(Common::cells[rowC + 1][colC].getCellSize());
      Common::megaCells[rowM][colM].setObstacle(obstacle);
      colM++;
    }
    rowM++;
    colM = 0;
  }
}

// Move robot with STC algorithm
void KobukiController::moveWithSTC() {
  // 500ms
  ros::Rate loop_rate(2);
  loop_rate.sleep();

  int row, col;
  int distanceX = currentCellX - left + 1;
  int distanceY = currentCellY - down + 1;
  if (distanceX % cellSize == 0)
    col = distanceX / cellSize - 1;
  else
    col = distanceX / cellSize;

  if (distanceY % cellSize == 0)
    row = distanceY / cellSize - 1;
  else
    row = distanceY / cellSize;

  stcNavigation.startCell = Common::cells[row][col];
  stcNavigation.startMegaCell = Common::findMegaCellByCell(stcNavigation.startCell);

  stcNavigation.currentCell = stcNavigation.startCell;
  stcNavigation.currentMegaCell = stcNavigation.startMegaCell;

  for(int i = Common::rowCells - 1; i >= 0; i--) {
    for(int j = 0; j < Common::colCells; j++) {
      if(i == row && j == col)
        printf("o");
      else if(Common::cells[i][j].hasObstacle())
        printf("x");
      else
        printf("-");

      if(j == (Common::colCells - 1))
        printf("\n");
    }
  }

  // Quet hang xom xung quanh megaCell hien tai
  MegaCell megaCell = stcNavigation.scanNeighbor(1);

  // Di chuyen lan dau tien
  if (stcNavigation.passedMegaCellPath.empty() && (megaCell == stcNavigation.currentMegaCell)) {
    // Ket thuc vi khong co duong di
    ROS_INFO("KHONG CO DUONG DI!");
    return;
  } else {
    ROS_INFO("DI CHUYEN LAN DAU");
    moveToMegaCell(megaCell);
  }

  // Di chuyen cho nhung lan tiep theo.
  // Dieu kien dung la Danh sach megaCellPath rong VA khong tim thay hang xom.
  megaCell = stcNavigation.scanNeighbor(0);
  int index = 0;
  while (!(stcNavigation.passedMegaCellPath.empty() && (megaCell == stcNavigation.currentMegaCell))) {
    moveToMegaCell(megaCell);
    megaCell = stcNavigation.scanNeighbor(0);
  }

  ROS_INFO("END MOVESTC");
}

// Move from currentMegaCell to megaCell
void KobukiController::moveToMegaCell(MegaCell megaCell) {
  Cell cell;
  // Kiem tra xem co duong di k?
  if (megaCell != stcNavigation.currentMegaCell) {
    stcNavigation.passedMegaCellPath.push(stcNavigation.currentMegaCell);
    stcNavigation.generatePath(megaCell);
  } else {
    megaCell = stcNavigation.passedMegaCellPath.top();
    stcNavigation.passedMegaCellPath.pop();
    stcNavigation.generatePath(megaCell);
  }

  // Bau dau di chuyen tu currentMegaCell sang megaCell
  while (!stcNavigation.passedCellPath.empty()) {
    cell = stcNavigation.passedCellPath.front();
    stcNavigation.passedCellPath.pop();
    int direction = stcNavigation.passedCellDirection.front();
    stcNavigation.passedCellDirection.pop();
    if (moveToCell(cell, direction)) {
      stcNavigation.currentCell = cell;
    }
  }

  // Gan currentMegaCell bang megaCell moi. Gan parentMegaCell, cho megaCell vao lich su
  stcNavigation.passedMegaCell.push_back(stcNavigation.currentMegaCell);
  stcNavigation.parentMegaCell = stcNavigation.currentMegaCell;
  stcNavigation.currentMegaCell = megaCell;
  stcNavigation.currentDirection = stcNavigation.passedCellDirection.front();
  stcNavigation.passedCellDirection.pop();
}

// bool KobukiController::initMoveBase() {
//   int i = 0;

//   //retry 2 mins
//   while ((!action_movebase_client.waitForServer(ros::Duration(5.0))) && (i < 24)) {
//     ROS_INFO("Waiting for the move_base action server to come up");
//     i++;
//   }

//   if (i < 24) {
//     ROS_INFO("move_base action server activated");
//     return true;
//   } else
//     return false;
// }

// Use movebase to move from currentCell to cell
bool KobukiController::moveToCell(Cell cell, int direction) {
  // if (!initMoveBase()) {
  //   return false;
  // }

// move_to:
//   //set up the frame parameters
//   move_base_goal.target_pose.header.frame_id = "map";
//   move_base_goal.target_pose.header.stamp = ros::Time::now();

//   move_base_goal.target_pose.pose.position.x = cell.getCentreX() * map.getResolution() +
//       map.getOriginX();
//   move_base_goal.target_pose.pose.position.y =  cell.getCentreY() * map.getResolution() +
//       map.getOriginY();

//   move_base_goal.target_pose.pose.position.z =  0.0;
//   move_base_goal.target_pose.pose.orientation.x = 0.0;
//   move_base_goal.target_pose.pose.orientation.y = 0.0;
//   move_base_goal.target_pose.pose.orientation.z = 0.0;
//   move_base_goal.target_pose.pose.orientation.w = 1.0;


//   action_movebase_client.sendGoal(move_base_goal);
//   action_movebase_client.waitForResult();

//   if (action_movebase_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
//     return true;
//   } else {
//     retry++;
//     if (retry <= maxRetry) {

//       // 100ms
//       ros::Rate retry_rate(10);
//       for (int i = 0; i < 10; i++) {
//         // Quay trai o day
//         retry_rate.sleep();
//       }
//       goto move_to;
//     } else {
//       return false;
//     }
//   }

  float distance = cell.getCellSize() * 0.05;

  switch (direction) {
  case D_UP:
    goStraight(distance);
    break;
  case D_LEFT:
    turnLeft();
    goStraight(distance);
    break;
  case D_RIGHT:
    turnRight();
    goStraight(distance);
    break;
  case D_DOWN:
    goForward(distance);
    break;
  }
}

void KobukiController::turn(int degree) {
  twist.linear.x = 0;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = angularSpeed;
  if (degree < 0)
    twist.angular.z = -angularSpeed;

  float relativeAngle = degree * PI / 180;

  clock_t start, end;
  start = clock();
  float currentAngle = 0;

  // Start rotate
  while (currentAngle < abs(relativeAngle)) {
    cmd_vel_pub.publish(twist);
    end = clock();
    currentAngle = angularSpeed * (double) ((end - start) / CLOCKS_PER_SEC);
  }

  // Stop
  twist.angular.z = 0;
  cmd_vel_pub.publish(twist);
  ROS_INFO("TURN SUCCEEDED");
}

void KobukiController::turnLeft() {
  ROS_INFO("TURN LEFT");
  turn(90);
}

void KobukiController::turnRight() {
  ROS_INFO("TURN RIGHT");
  turn(-90);
}

void KobukiController::go(float distance) {
  twist.linear.x = linearSpeed;
  twist.linear.y = 0;
  twist.linear.z = 0;
  twist.angular.x = 0;
  twist.angular.y = 0;
  twist.angular.z = 0;

  time_t start = time(0);
  float currentDistance = 0;

  // Start go
  while (currentDistance < distance) {
    cmd_vel_pub.publish(twist);
    time_t end = time(0);
    currentDistance = linearSpeed * (end - start);
  }

  // Stop
  twist.linear.x = 0;
  cmd_vel_pub.publish(twist);
  ROS_INFO("GO SUCCEEDED");
}

void KobukiController::goStraight(float distance) {
  ROS_INFO("GO STRAIGHT");
  go(distance);
}

void KobukiController::goForward(float distance) {
  ROS_INFO("GO FORWARD");
  turn(180);
  go(distance);
}

void KobukiController::bumperEventHandle(const kobuki_msgs::BumperEventConstPtr msg) {
  return;
}

void KobukiController::bumperEventHandle(const kobuki_msgs::CliffEventConstPtr msg) {
  return;
}

void KobukiController::odometryHandle(const nav_msgs::OdometryConstPtr odometry) {
  return;
}

void KobukiController::laserHandle(const sensor_msgs::LaserScanConstPtr laser) {
  return;
}

// Publish vi tri robot len rviz de view.
void KobukiController::amclHandle(const geometry_msgs::PoseWithCovarianceStampedConstPtr amclpose) {
  maxAMCL++;
  if (maxAMCL < 100) {
    amcl_pub.publish(amclpose);
    currentCellX = (amclpose->pose.pose.position.x - map.getOriginX()) / map.getResolution();
    currentCellY = (amclpose->pose.pose.position.y - map.getOriginY()) / map.getResolution();
  }
}
