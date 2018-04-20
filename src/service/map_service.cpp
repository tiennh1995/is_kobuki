#include "../../include/service/map_service.hpp"

bool MapService::init() {
  ROS_INFO("Init MapService");
  // Subscriber
  // Init map
  map_metadata_sub = nh.subscribe("/map_metadata", 10,
                                  &MapService::initializeMap, this);

  // Services
  update_robot_service = nh.advertiseService("update_robot", &MapService::updateRobot, this);
  update_map_service = nh.advertiseService("update_map", &MapService::updateMap, this);
  valid_mega_cells_service = nh.advertiseService("valid_mega_cells", &MapService::validMegaCells, this);

  ROS_INFO("Init MapService success!");

  ros::Rate loop_rate(60);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return true;
}

// Init map
void MapService::initializeMap(const nav_msgs::MapMetaDataConstPtr msg) {
  if (initMap) {
    map.setHeight(msg->height);
    map.setWidth(msg->width);
    map.setResolution(msg->resolution);
    map.setOriginX(msg->origin.position.x);
    map.setOriginY(msg->origin.position.y);

    map_sub = nh.subscribe("/map", 10,
                           &MapService::setMapData, this);
    initMap = false;
  }
}

void MapService::setMapData(const nav_msgs::OccupancyGridConstPtr msg) {
  std::vector<int> mapData;

  for (int i = 0; i < msg->data.size(); i++) {
    mapData.push_back((int) msg->data[i]);
  }

  map.setMapData(map.oneArrToTwoArr(mapData));
  initCell();
}

// Khoi tao cell, megaCell
void MapService::initCell() {
  int up = map.findLineUp();
  int left = map.findLineLeft();
  int down = map.findLineDown();
  int right = map.findLineRight();

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
}

bool MapService::updateMap(is_kobuki::UpdateMap::Request& request,
                           is_kobuki::UpdateMap::Response& response) {
  ROS_INFO("Update Map");

  int robotId = (int) request.robotId;
  int row = (int) request.row;
  int col = (int) request.col;
  bool result = false;

  if (validRobotId(robotId) && validCell(row, col)) {
    Cell cell = Common::cells[row][col];

    cell.setStatus((int) request.status);
    MegaCell megaCell = Common::findMegaCellByCell(cell);
    if (!megaCell.isNULL() && megaCell.isScaned()) {
      megaCell.setStatus((int) request.status);
      result = true;
    }
  }

  response.result = result;
  if (result)
    ROS_INFO("Update Map Success!");
  else
    ROS_INFO("Update Map Fail!");

  return true;
}

bool MapService::validMegaCells(is_kobuki::ValidMegaCells::Request& request,
                                is_kobuki::ValidMegaCells::Response& response) {
  ROS_INFO("Get valid megaCells");

  int robotId = (int) request.robotId;
  int row = 0, col = 0;
  bool result;

  if (validRobotId(robotId) && robotId <= robotSize) {
    if (robotSize > 1 && robots[robotSize - robotId].getStatus()) {
      row = robots[robotSize - robotId].getMaxRow();
      col = robots[robotSize - robotId].getMaxCol();
    }

    for (int i = row; i < Common::rowCells / 2; i++)
      for (int j = col; j < Common::colCells / 2; j++)
        if (Common::megaCells[i][j].getStatus() == WAITING &&
            !Common::megaCells[i][j].hasObstacle()) {
          response.rows.push_back(i);
          response.cols.push_back(j);
        }
    result = true;
  }

  response.result = result;
  if (result)
    ROS_INFO("Get valid megaCells Success!");
  else
    ROS_INFO("Get valid megaCells Fail!");

  return true;
}

bool MapService::updateRobot(is_kobuki::UpdateRobot::Request& request,
                             is_kobuki::UpdateRobot::Response& response) {
  ROS_INFO("Update Robot");

  bool init = (bool) request.init;
  int robotId = (int) request.robotId;
  bool result = false;

  if (validRobotId(robotId)) {
    if (init && robotSize < MAX_ROBOT_SIZE) {
      Robot robot = robots[robotSize];
      robotSize++;

      robot.setId(robotId);
      robot.setStatus(true);
      robot.setMinRow(0);
      robot.setMaxRow(Common::rowCells / 2);

      if (robotSize > 1) {
        robot.setMinCol(0);
        robot.setMaxCol(Common::colCells / 4);
      } else {
        robot.setMinCol(Common::colCells / 4);
        robot.setMaxCol(Common::colCells / 2);
      }

      response.minRow = robot.getMinRow();
      response.maxRow = robot.getMaxRow();
      response.minCol = robot.getMinCol();
      response.maxCol = robot.getMaxCol();
      result = true;
    } else {
      int index = findRobot(robotId);

      if (index >= 0) {
        robots[index].setStatus((bool) request.status);
        result = true;
      }
    }
  }

  response.result = result;
  if (result)
    ROS_INFO("Update Robot Success!");
  else
    ROS_INFO("Update Robot Fail!");

  return true;
}

int MapService::findRobot(int robotId) {
  for (int i = 0; i < robotSize; i++)
    if (robots[i].getId() == robotId)
      return i;

  return -1;
}

bool MapService::validRobotId(int robotId) {
  return 1 <= robotId && robotId <= MAX_ROBOT_SIZE;
}

bool MapService::validCell(int row, int col) {
  return 0 <= row && row < Common::rowCells && 0 <= col && col < Common::colCells;
}

