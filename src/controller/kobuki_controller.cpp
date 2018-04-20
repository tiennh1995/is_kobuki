#include "../../include/controller/kobuki_controller.hpp"

int KobukiController::getRobotId() {
  return robotId;
}

void KobukiController::setRobotId(int robotId) {
  this->robotId = robotId;
}

bool KobukiController::getIsDirectionX() {
  return isDirectionX;
}

void KobukiController::setIsDirectionX(bool isDirectionX) {
  this->isDirectionX = isDirectionX;
}

bool KobukiController::init() {
  ROS_INFO("Init Controller");

  // Subscriber
  // Init map
  amcl_sub = nh.subscribe(initTopic("/amcl_pose"), 1,
                          &KobukiController::amclHandle, this);
  map_metadata_sub = nh.subscribe("/map_metadata", 10,
                                  &KobukiController::initializeMap, this);
  bumper_event_sub = nh.subscribe(initTopic("/mobile_base/events/bumper"), 1,
                                  &KobukiController::bumperEventHandle, this);
  cliff_event_sub = nh.subscribe(initTopic("/mobile_base/events/cliff"), 1,
                                 &KobukiController::bumperEventHandle, this);
  odometry_sub = nh.subscribe(initTopic("/odom"), 1,
                              &KobukiController::odometryHandle, this);
  laser_sub = nh.subscribe(initTopic("/scan"), 1, &KobukiController::laserHandle, this);

  // Services
  updateRobotClient = nh.serviceClient<is_kobuki::UpdateRobot>("/update_robot");
  updateMapClient = nh.serviceClient<is_kobuki::UpdateMap>("/update_map");
  validMegaCellsClient = nh.serviceClient<is_kobuki::ValidMegaCells>("/valid_mega_cells");

  // Publisher
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(initTopic("/mobile_base/commands/velocity"), 1);
  power_pub = nh.advertise<kobuki_msgs::MotorPower>(initTopic("/mobile_base/commands/motor_power"), 1);

  ROS_INFO("Init Controller Success!");
  ros::spin();

  return true;
}

std::string KobukiController::initTopic(std::string topic) {
  std::stringstream stream;
  if (robotId != 0)
    stream << "/mobile_base" << robotId << topic;
  else
    stream << topic;

  return stream.str();
}

// Init map
void KobukiController::initializeMap(const nav_msgs::MapMetaDataConstPtr msg) {
  if (initMap) {
    map.setHeight(msg->height);
    map.setWidth(msg->width);
    map.setResolution(msg->resolution);
    map.setOriginX(msg->origin.position.x);
    map.setOriginY(msg->origin.position.y);

    map_sub = nh.subscribe("/map", 10, &KobukiController::setMapData, this);
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

  thread.start(&KobukiController::moveWithSTC, *this);
  updateRobot(true, true);
}

// Khoi tao cell, megaCell
void KobukiController::initCell() {
  up = map.findLineUp();
  left = map.findLineLeft();
  down = map.findLineDown();
  right = map.findLineRight();

  // Khoi tao mang cells

  // Tinh so dong sau khi thu nho
  int numberRow = ((up - down + 1) / cellSize);
  // Lam tron sao cho so dong chia het cho 2, de thuan tien cho viec chia megaCell
  if (numberRow % 2 != 0)
    numberRow--;

  // Tinh so cot sau khi thu nho
  int numberCol = (right - left + 1) / cellSize;
  // Lam tron sao cho so cot chia het cho 2, de thuan tien cho viec chia megaCell
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

  ROS_INFO("Cell: ");
  for (int i = Common::rowCells - 1; i >= 0; i--)
    for (int j = 0; j < Common::colCells; j++) {
      if (Common::cells[i][j].getStatus() == SCANED)
        printf("o");
      else if (Common::cells[i][j].hasObstacle())
        printf("x");
      else
        printf("-");

      if (j == Common::colCells - 1)
        printf("\n");
    }

  ROS_INFO("MegaCell: ");
  for (int i = Common::rowCells / 2 - 1; i >= 0; i--)
    for (int j = 0; j < Common::colCells / 2; j++) {
      if (Common::megaCells[i][j].getStatus() == SCANED)
        printf("o");
      else if (Common::megaCells[i][j].hasObstacle())
        printf("x");
      else
        printf("-");

      if (j == Common::colCells / 2 - 1)
        printf("\n");
    }
}

// Move robot with STC algorithm
void KobukiController::moveWithSTC() {
  ROS_INFO("START MOVE STC");
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

  // Quet hang xom xung quanh megaCell hien tai
  MegaCell megaCell = stcNavigation.scanNeighbor(1);

  // Di chuyen lan dau tien
  if (stcNavigation.passedMegaCellPath.empty() && (megaCell == stcNavigation.currentMegaCell)) {
    // Ket thuc vi khong co duong di
    ROS_INFO("NO VALID PATH!");
    return;
  } else {
    moveToMegaCell(megaCell);
  }

  // Di chuyen cho nhung lan tiep theo.
  // Dieu kien dung la Danh sach megaCellPath rong VA khong tim thay hang xom.
  megaCell = stcNavigation.scanNeighbor(0);
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
}

// Use movebase to move from currentCell to cell
bool KobukiController::moveToCell(Cell cell, int direction) {
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

  reMove();
  cell.setStatus(SCANED);
  int *index = Common::findIndexCell(cell);
  Common::cells[index[0]][index[1]].setStatus(SCANED);

  system("clear");
  ROS_INFO("Cell: ");
  for (int i = Common::rowCells - 1; i >= 0; i--)
    for (int j = 0; j < Common::colCells; j++) {
      if (Common::cells[i][j].getStatus() == SCANED)
        printf("o");
      else if (Common::cells[i][j].hasObstacle())
        printf("x");
      else
        printf("-");

      if (j == Common::colCells - 1)
        printf("\n");
    }

  ROS_INFO("MegaCell: ");
  for (int i = Common::rowCells / 2 - 1; i >= 0; i--)
    for (int j = 0; j < Common::colCells / 2; j++) {
      if (Common::megaCells[i][j].getStatus() == SCANED)
        printf("o");
      else if (Common::megaCells[i][j].hasObstacle())
        printf("x");
      else
        printf("-");

      if (j == Common::colCells / 2 - 1)
        printf("\n");
    }

  updateMap(cell);
  return true;
}

void KobukiController::turn(float degree) {
  ros::Rate rate(10);
  float theta = current_pose.theta;
  float x = current_pose.x;
  float y = current_pose.y;
  geometry_msgs::Twist move;
  if (degree > 0)
    move.angular.z = angularSpeed;
  else
    move.angular.z = -angularSpeed;

  while (ros::ok() && (std::abs(current_pose.theta - theta) < std::abs(degree))) {
    cmd_vel_pub.publish(move);

    ros::spinOnce();
    rate.sleep();
  }

  setIsDirectionX(!isDirectionX);
  deltaTheta += current_pose.theta - theta - degree;
  deltaX += current_pose.x - x;
  deltaY += current_pose.y - y;
  usleep(1000000);
  ROS_INFO("TURN SUCCEEDED");
}

void KobukiController::turnLeft() {
  ROS_INFO("TURN LEFT");
  turn(PI_MOVE / 2);
}

void KobukiController::turnRight() {
  ROS_INFO("TURN RIGHT");
  turn(-PI_MOVE / 2);
}

void KobukiController::go(float distance) {
  ros::Rate rate(10);
  float theta = current_pose.theta;
  float x = current_pose.x;
  float y = current_pose.y;
  if (getIsDirectionX()) {
    geometry_msgs::Twist move;
    move.linear.x = linearSpeed;
    while (ros::ok() && (std::abs(current_pose.x - x) < distance)) {
      cmd_vel_pub.publish(move);
      ros::spinOnce();
      rate.sleep();
    }

    deltaX += current_pose.x - x - distance;
    deltaY += current_pose.y - y;
  } else {
    geometry_msgs::Twist move;
    move.linear.x = linearSpeed;
    while (ros::ok() && (std::abs(current_pose.y - y) < distance)) {
      cmd_vel_pub.publish(move);
      ros::spinOnce();
      rate.sleep();
    }

    deltaX += current_pose.x - x;
    deltaY += current_pose.y - y - distance;
  }

  deltaTheta += current_pose.theta - theta;
  usleep(1000000);
  ROS_INFO("GO SUCCEEDED");
}

void KobukiController::goStraight(float distance) {
  ROS_INFO("GO STRAIGHT");
  go(distance);
}

void KobukiController::goForward(float distance) {
  ROS_INFO("GO FORWARD");
  turn(PI_MOVE / 2);
  turn(PI_MOVE / 2);
  go(distance);
}

void KobukiController::reMove() {
  ROS_INFO("REMOVE");
  if (std::abs(deltaTheta) >= epsilonTheta) {
    if (deltaTheta > 0) {
      turn(-(PI_MOVE / 2 + deltaTheta));
      getIsDirectionX() ? go(std::abs(deltaX)) : go(std::abs(deltaY));
      turn(PI_MOVE / 2);
    } else {
      turn(PI_MOVE / 2 - deltaTheta);
      getIsDirectionX() ? go(std::abs(deltaX)) : go(std::abs(deltaY));
      turn(-PI_MOVE / 2);
    }
  }

  if (std::abs(deltaX) >= epsilonX)
    go(deltaX);

  if (std::abs(deltaY) >= epsilonY)
    go(deltaY);

  ROS_INFO("REMOVE SUCCESSED");
}

int KobukiController::updateMap(Cell cell) {
  ROS_INFO("Update Map");

  is_kobuki::UpdateMap msg;
  bool result = false;

  int* index = Common::findIndexCell(cell);
  msg.request.col = index[0];
  msg.request.row = index[1];
  msg.request.status = cell.getStatus();
  updateMapClient.call(msg);
  result = (bool) msg.response.result;

  if (result)
    ROS_INFO("Update Map Success!");
  else
    ROS_INFO("Update Map Fail!");

  return 0;
}

void KobukiController::validMegaCells(int robotId) {
  ROS_INFO("Get ValidMegaCells");

  is_kobuki::ValidMegaCells msg;
  bool result = false;

  msg.request.robotId = robotId;
  validMegaCellsClient.call(msg);
  result = (bool) msg.response.result;

  if (result) {
    std::vector<int> rows = (std::vector<int>) msg.response.rows;
    std::vector<int> cols = (std::vector<int>) msg.response.cols;

    for (int i = 0; i < rows.size(); i++)
      Common::megaCells[rows.at(i)][cols.at(i)].setObstacle(false);

    ROS_INFO("Update Map Success!");
  } else
    ROS_INFO("Update Map Fail!");
}

int KobukiController::updateRobot(bool init, bool status) {
  ROS_INFO("Update Robot");

  is_kobuki::UpdateRobot msg;
  bool result = false;

  msg.request.robotId = robotId;
  msg.request.init = init;
  msg.request.status = status;
  updateRobotClient.call(msg);
  result = (bool) msg.response.result;

  if (result) {
    if (init) {
      int minRow = (int) msg.response.minRow;
      int maxRow = (int) msg.response.maxRow;
      int minCol = (int) msg.response.minCol;
      int maxCol = (int) msg.response.maxCol;
      for (int row = minRow; row < maxRow; row++)
        for (int col = minCol; col < maxCol; col++)
          Common::megaCells[row][col].setObstacle(true);
    }

    ROS_INFO("Update Robot Success!");
  } else
    ROS_INFO("Update Robot Fail!");

  return 0;
}

void KobukiController::odometryHandle(const nav_msgs::OdometryConstPtr& odometry) {
  // linear position
  current_pose.x = odometry->pose.pose.position.x;
  current_pose.y = odometry->pose.pose.position.y;

  // quaternion to RPY conversion
  tf::Quaternion q(
    odometry->pose.pose.orientation.x,
    odometry->pose.pose.orientation.y,
    odometry->pose.pose.orientation.z,
    odometry->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // angular position
  current_pose.theta = yaw;
}

void KobukiController::bumperEventHandle(const kobuki_msgs::BumperEventConstPtr msg) {
  return;
}

void KobukiController::cliffEventHandle(const kobuki_msgs::CliffEventConstPtr msg) {
  return;
}

void KobukiController::laserHandle(const sensor_msgs::LaserScanConstPtr laser) {
  return;
}

// Publish vi tri robot len rviz de view.
void KobukiController::amclHandle(const geometry_msgs::PoseWithCovarianceStampedConstPtr amclpose) {
  maxAMCL++;
  if (maxAMCL < 100) {
    ROS_INFO("%f, %f", amclpose->pose.pose.position.x, amclpose->pose.pose.position.y);
    amcl_pub.publish(amclpose);
    currentCellX = (amclpose->pose.pose.position.x - map.getOriginX()) / map.getResolution();
    currentCellY = (amclpose->pose.pose.position.y - map.getOriginY()) / map.getResolution();
  }
}
