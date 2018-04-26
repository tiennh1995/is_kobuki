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
  updateMapClient = nh.serviceClient<is_kobuki::UpdateMap>("/update_map");
  validMegaCellsClient = nh.serviceClient<is_kobuki::ValidMegaCells>("/valid_mega_cells");
  divideMapClient = nh.serviceClient<is_kobuki::DivideMap>("/divide_map");

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
}

// Khoi tao cell, megaCell
void KobukiController::initCell() {
  up = map.findLineUp();
  left = map.findLineLeft();
  down = map.findLineDown();
  right = map.findLineRight();

  // Khoi tao mang cells

  // Tinh so dong sau khi thu nho
  int numberRow = ((up - down + 1) / cellSize) + 1;
  // Lam tron sao cho so dong chia het cho 2, de thuan tien cho viec chia megaCell
  if (numberRow % 2 != 0)
    numberRow--;

  // Tinh so cot sau khi thu nho
  int numberCol = (right - left + 1) / cellSize + 1;
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

  // chia map trong truong hop chay 2 robot
  if (robotId != 0) {
    divideMap(robotId);
    for (int rowC = 0; rowC < numberRow; rowC++) {
      for (int colC = 0; colC < numberCol; colC++) {
        if (!(rowC >= minRow && rowC <= maxRow && colC >= minCol && colC <= maxCol)) {
          Common::cells[rowC][colC].setObstacle(true);
        }
      }
      col = left;
      row = row + cellSize;
    }
  }

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

  printCellAndMegaCell();
}

// Move robot with STC algorithm
void KobukiController::moveWithSTC() {

  ROS_INFO("START MOVE STC");
  // 500ms
  ros::Rate loop_rate(2);
  loop_rate.sleep();
  // for(int k=0;k<500000000;k++){}

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

  row = 11;
  col = 2;
  stcNavigation.startCell = Common::cells[row][col];
  stcNavigation.startMegaCell = Common::findMegaCellByCell(stcNavigation.startCell);

  stcNavigation.currentCell = stcNavigation.startCell;
  stcNavigation.currentMegaCell = stcNavigation.startMegaCell;
  Common::cells[row][col].setStatus(SCANED);
  if (robotId != 0) updateMap(Common::cells[row][col], 0);

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

  if (robotId != 0) {
    int robotStatus = updateMap(stcNavigation.currentCell, 1);
    while (robotStatus != 2) {
      if (robotStatus == 0) {
        if (!validMegaCells(robotId)) break;
        printCellAndMegaCell();

        goToCertainCell(Common::cells, stcNavigation.currentCell, certainCell);
        int *temp = Common::findIndexCell(certainCell);
        int row = temp[0];
        int col = temp[1];
        stcNavigation.currentCell = Common::cells[row][col];
        stcNavigation.currentMegaCell = Common::findMegaCellByCell(stcNavigation.currentCell);
        Common::cells[row][col].setStatus(SCANED);
        if (robotId != 0) updateMap(Common::cells[row][col], 0);

        MegaCell megaCell = stcNavigation.scanNeighbor(1);

        if (stcNavigation.passedMegaCellPath.empty() && (megaCell == stcNavigation.currentMegaCell)) {
          ROS_INFO("NO VALID PATH!");
          return;
        } else {
          moveToMegaCell(megaCell);
        }
        megaCell = stcNavigation.scanNeighbor(0);
        while (!(stcNavigation.passedMegaCellPath.empty() && (megaCell == stcNavigation.currentMegaCell))) {
          moveToMegaCell(megaCell);
          megaCell = stcNavigation.scanNeighbor(0);
        }

        ROS_INFO("END MOVESTC");
      }
      robotStatus = updateMap(stcNavigation.currentCell, 1);
      usleep(2000000);
    }
  }
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
  float distance = cell.getCellSize() * 0.0213;

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

  cell.setStatus(SCANED);
  int *index = Common::findIndexCell(cell);
  Common::cells[index[0]][index[1]].setStatus(SCANED);
  MegaCell megaCell = Common::findMegaCellByCell(cell);
  int *temp = Common::findIndexMegaCell(megaCell);
  if (Common::megaCells[temp[0]][temp[1]].isScaned() == SCANED)
    Common::megaCells[temp[0]][temp[1]].setStatus(SCANED);

  system("clear");
  printCellAndMegaCell();

  if (robotId != 0)
    updateMap(cell, 0);

  return true;
}

void KobukiController::turn(float degree) {
  ros::Rate rate(50);
  float theta = current_pose.theta;
  float current_theta = current_pose.theta;
  geometry_msgs::Twist move;
  move.angular.z = degree > 0 ? angularSpeed : -angularSpeed;

  while (ros::ok() && (std::abs(current_theta - theta) < std::abs(degree))) {
    cmd_vel_pub.publish(move);
    ros::spinOnce();
    rate.sleep();
    current_theta = current_pose.theta;
    if (std::abs(current_theta - theta) > (PI + 0.5)) {
      theta = -theta;
      if (degree < 0)
        degree += std::abs(PI) - std::abs(theta);
      else
        degree -= std::abs(PI) - std::abs(theta);
    }
  }

  // stop
  move.angular.z = 0;
  cmd_vel_pub.publish(move);

  setIsDirectionX(!isDirectionX);
  usleep(2000000);
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
  ros::Rate rate(50);
  float theta = current_pose.theta;
  float x = current_pose.x;
  float y = current_pose.y;
  geometry_msgs::Twist move;
  move.linear.x = linearSpeed;

  if (getIsDirectionX()) {
    while (ros::ok() && (std::abs(current_pose.x - x) < distance)) {
      cmd_vel_pub.publish(move);
      ros::spinOnce();
      rate.sleep();
    }
  } else {
    while (ros::ok() && (std::abs(current_pose.y - y) < distance)) {
      cmd_vel_pub.publish(move);
      ros::spinOnce();
      rate.sleep();
    }
  }

  usleep(2000000);
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


int KobukiController::updateMap(Cell cell) {
  ROS_INFO("Update Map");

  is_kobuki::UpdateMap msg;
  bool result = false;

  int* index = Common::findIndexCell(cell);
  msg.request.row = index[0];
  msg.request.col = index[1];
  msg.request.status = cell.getStatus();
  msg.request.isFinish = isFinish;
  msg.request.robotId = robotId;
  updateMapClient.call(msg);
  result = (bool) msg.response.result;

  if (result)
    ROS_INFO("Update Map Success!");
  else
    ROS_INFO("Update Map Fail!");

  return (int)msg.response.robotStatus;
}

bool KobukiController::validMegaCells(int robotId) {
  ROS_INFO("Get ValidMegaCells");

  is_kobuki::ValidMegaCells msg;
  bool result = false;

  msg.request.robotId = robotId;
  validMegaCellsClient.call(msg);
  result = (bool) msg.response.result;

  if (result) {
    rows = (std::vector<int>) msg.response.rows;
    cols = (std::vector<int>) msg.response.cols;

    for (int i = 0; i < rows.size(); i++) {
      Common::megaCells[rows.at(i)][cols.at(i)].setObstacle(false);
      Cell *cells = Common::megaCells[rows.at(i)][cols.at(i)].getCells();
      int *temp0 = Common::findIndexCell(cells[0]);
      Common::cells[temp0[0]][temp0[1]].setObstacle(false);
      int *temp1 = Common::findIndexCell(cells[1]);
      Common::cells[temp1[0]][temp1[1]].setObstacle(false);
      int *temp2 = Common::findIndexCell(cells[2]);
      Common::cells[temp2[0]][temp2[1]].setObstacle(false);
      int *temp3 = Common::findIndexCell(cells[3]);
      Common::cells[temp3[0]][temp3[1]].setObstacle(false);
      certainCell = cells[3];
    }

    ROS_INFO("Has ValidMegaCells!");
  } else
    ROS_INFO("No ValidMegaCells!");
  return result;
}

void KobukiController::divideMap(int robotId) {
  ROS_INFO("Update Robot");

  is_kobuki::DivideMap msg;
  msg.request.robotId = robotId;
  divideMapClient.call(msg);
  minRow = (int) msg.response.minRow;
  maxRow = (int) msg.response.maxRow;
  minCol = (int) msg.response.minCol;
  maxCol = (int) msg.response.maxCol;

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

void KobukiController::printCellAndMegaCell() {
  for (int i = Common::rowCells - 1; i >= 0; i--)
    for (int j = 0; j < Common::colCells; j++) {
      if (Common::cells[i][j].getStatus() == SCANED)
        printf("o ");
      else if (Common::cells[i][j].hasObstacle())
        printf("x ");
      else if (Common::cells[i][j].getStatus() == MOVING)
        printf("* ");
      else
        printf("- ");

      if (j == Common::colCells - 1)
        printf("\n");
    }


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

void KobukiController::goToCertainCell(Cell **cells, Cell beginCell, Cell certainCell) {

  int **B;   // sua
  B = new int*[Common::rowCells];
  for (int i = 0; i < Common::rowCells; i++)
    B[i] = new int[Common::colCells];
  int C[80][80];
  int D[80];
  int i, j, a, b, c, d, e, f, g, h, k, n, m;
  int o, p, q;
  int dodai;
  int chuoi;
  int so;
  int hang;
  int hang2;
  int cot;
  int lap;
  int dodai1;
  printf("nhap mang\n");

  for (i = 0; i <= Common::rowCells - 1; i++) {
    for (j = 0; j <= Common::colCells - 1; j++) {
      if (cells[i][j].hasObstacle()) {
        printf("x ");
        B[i][j] = 2;
      } else {
        printf("_ ");
        B[i][j] = 0;
      }
    }
    printf("\n");
  }
  int *index = Common::findIndexCell(beginCell);
  a = index[0];
  b = index[1];
  int *temp = Common::findIndexCell(certainCell);
  c = temp[0];
  d = temp[1];
  printf("%d %d --> %d %d\n", a, b, c, d);

  a = Common::rowCells - 1 - a;
  c = Common::rowCells - 1 - c;
  if (a != c || b != d) {
    e = a + 1;
    f = a - 1;
    g = b + 1;
    h = b - 1;
    if ( B[e][b] == 0) {
      B[e][b] = 1;
      C[0][0] = 1;
      C[0][1] = e * 100 + b;
    }
    if ( B[f][b] == 0) {
      B[f][b] = 1;
      C[1][0] = 1;
      C[1][1] = f * 100 + b;
    }
    if ( B[a][g] == 0) {
      B[a][g] = 1;
      C[2][0] = 1;
      C[2][1] = a * 100 + g;
    }
    if ( B[a][h] == 0) {
      B[a][h] = 1;
      C[3][0] = 1;
      C[3][1] = a * 100 + h;
    }

  }
  while (p != c || q != d) {
    dodai = dodai + 1;
    for (chuoi = 0; chuoi <= 30; chuoi++) {
      if (C[chuoi][0] == dodai) {
        so = C[chuoi][dodai];
        hang = so / 100;
        cot = so % 100;
        if (hang == c && cot == d) {
          p = hang;
          q = cot;
          for (k = 1; k <= dodai; k++) {
            D[k] = C[chuoi][k];
          }
        }
        e = hang + 1;
        f = hang - 1;
        g = cot + 1;
        h = cot - 1;
        dodai1 = dodai + 1;
        if (B[e][cot] == 0) {
          lap = 0;
          B[e][cot] = 1;
          while (C[lap][0] != 0) {
            lap = lap + 1;
          }
          C[lap][0] = dodai1;
          C[lap][dodai1] = e * 100 + cot;
          for (k = 1; k <= dodai; k++) {
            C[lap][k] = C[chuoi][k];
          }
        }
        if (B[f][cot] == 0) {
          lap = 0;
          B[f][cot] = 1;
          while (C[lap][0] != 0) {
            lap = lap + 1;
          }
          C[lap][0] = dodai1;
          C[lap][dodai1] = f * 100 + cot;
          for (k = 1; k <= dodai; k++) {
            C[lap][k] = C[chuoi][k];
          }
        }
        if (B[hang][g] == 0) {
          lap = 0;
          B[hang][g] = 1;
          while (C[lap][0] != 0) {
            lap = lap + 1;
          }
          C[lap][0] = dodai1;
          C[lap][dodai1] = hang * 100 + g;
          for (k = 1; k <= dodai; k++) {
            C[lap][k] = C[chuoi][k];
          }
        }
        if (B[hang][h] == 0) {
          lap = 0;
          B[hang][h] = 1;
          while (C[lap][0] != 0) {
            lap = lap + 1;
          }
          C[lap][0] = dodai1;
          C[lap][dodai1] = hang * 100 + h;
          for (k = 1; k <= dodai; k++) {
            C[lap][k] = C[chuoi][k];
          }
        }
        C[chuoi][0] = 0;
      }

    }
  }
  // printf("%d_%d\n", Common::rowCells - 1- a,b);
  printf("A\n");
  cellArr.push_back(Common::cells[Common::rowCells - 1 - a][b]);
  for (k = 1; k <= dodai; k++) {
    so = D[k];
    hang = so / 100;
    hang2 = Common::rowCells - 1 - hang;
    cot = so % 100;
    // printf("%d_%d\n", hang2,cot );
    cellArr.push_back(Common::cells[hang2][cot]);
  }

  int x = 0;
  while (!stcNavigation.passedCellDirection.empty())
    stcNavigation.passedCellDirection.pop();

  stcNavigation.currentCell = cellArr[x];
  x++;

  do {
    Cell cell = cellArr[x];
    x++;
    int position = stcNavigation.currentCell.getNeighbor(cell);
    stcNavigation.currentCell = cellArr[x - 1];
    switch (stcNavigation.currentDirection) {
    case D_UP:
      if (position == UP) {
        stcNavigation.passedCellDirection.push(D_UP);
        stcNavigation.currentDirection = D_UP;
      }
      if (position == LEFT) {
        stcNavigation.passedCellDirection.push(D_LEFT);
        stcNavigation.currentDirection = D_LEFT;
      }
      if (position == DOWN) {
        stcNavigation.passedCellDirection.push(D_DOWN);
        stcNavigation.currentDirection = D_DOWN;
      }
      if (position == RIGHT) {
        stcNavigation.passedCellDirection.push(D_RIGHT);
        stcNavigation.currentDirection = D_RIGHT;
      }
      break;
    case D_LEFT:
      if (position == UP) {
        stcNavigation.passedCellDirection.push(D_RIGHT);
        stcNavigation.currentDirection = D_UP;
      }
      if (position == LEFT) {
        stcNavigation.passedCellDirection.push(D_UP);
        stcNavigation.currentDirection = D_LEFT;
      }
      if (position == DOWN) {
        stcNavigation.passedCellDirection.push(D_LEFT);
        stcNavigation.currentDirection = D_DOWN;
      }
      if (position == RIGHT) {
        stcNavigation.passedCellDirection.push(D_DOWN);
        stcNavigation.currentDirection = D_RIGHT;
      }
      break;
    case D_DOWN:
      if (position == UP) {
        stcNavigation.passedCellDirection.push(D_DOWN);
        stcNavigation.currentDirection = D_UP;
      }
      if (position == LEFT) {
        stcNavigation.passedCellDirection.push(D_RIGHT);
        stcNavigation.currentDirection = D_LEFT;
      }
      if (position == DOWN) {
        stcNavigation.passedCellDirection.push(D_UP);
        stcNavigation.currentDirection = D_DOWN;
      }
      if (position == RIGHT) {
        stcNavigation.passedCellDirection.push(D_LEFT);
        stcNavigation.currentDirection = D_RIGHT;
      }
      break;
    case D_RIGHT:
      if (position == UP) {
        stcNavigation.passedCellDirection.push(D_LEFT);
        stcNavigation.currentDirection = D_UP;
      }
      if (position == LEFT) {
        stcNavigation.passedCellDirection.push(D_DOWN);
        stcNavigation.currentDirection = D_LEFT;
      }
      if (position == DOWN) {
        stcNavigation.passedCellDirection.push(D_RIGHT);
        stcNavigation.currentDirection = D_DOWN;
      }
      if (position == RIGHT) {
        stcNavigation.passedCellDirection.push(D_UP);
        stcNavigation.currentDirection = D_RIGHT;
      }
      break;
    }
  } while (x != cellArr.size());

  x = 1;
  stcNavigation.currentCell = cellArr[0];
  while (!stcNavigation.passedCellDirection.empty()) {
    int direction = stcNavigation.passedCellDirection.front();
    stcNavigation.passedCellDirection.pop();
    if (moveToCell(cellArr[x], direction)) {
      int *index = Common::findIndexCell(cellArr[x]);
      Common::cells[index[0]][index[1]].setStatus(MOVING);
      stcNavigation.currentCell = cellArr[x];
      x++;
    }
  }
}
