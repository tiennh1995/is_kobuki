#include "../../include/service/map_service.hpp"
bool MapService::init() {
  ROS_INFO("Init MapService");
  // Subscriber
  // Init map
  map_metadata_sub = nh.subscribe("/map_metadata", 10,
                                  &MapService::initializeMap, this);

  // Services
  update_map_service = nh.advertiseService("update_map", &MapService::updateMap, this);
  valid_mega_cells_service = nh.advertiseService("valid_mega_cells", &MapService::validMegaCells, this);
  divide_map_service = nh.advertiseService("divide_map", &MapService::divideMap, this);

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
  int duration;

  if (robotId == 1) {
    if (request.isFinish == 1)
      robots[0].setStatus(FINISH);

    robots[0].setUpdateTime();
    duration = (double)(robots[0].getUpdateTime() - robots[1].getUpdateTime());
    if (duration > timeDied && robots[1].getStatus() != FINISH)
      robots[1].setStatus(FAIL);

    response.robotStatus = robots[1].getStatus();
  } else if (robotId == 2) {
    if (request.isFinish == 1)
      robots[1].setStatus(FINISH);

    robots[1].setUpdateTime();
    duration = (double)(robots[1].getUpdateTime() - robots[0].getUpdateTime());
    if (duration > timeDied && robots[0].getStatus() != FINISH)
      robots[0].setStatus(FAIL);

    response.robotStatus = robots[0].getStatus();
  } else {
    ROS_INFO("ERROR robotId: %d", robotId);
    return false;
  }

  Common::cells[row][col].setStatus((int) request.status);
  MegaCell megaCell = Common::findMegaCellByCell(Common::cells[row][col]);
  if (!megaCell.isNULL() && megaCell.isScaned()) {
    int *index = Common::findIndexMegaCell(megaCell);
    Common::megaCells[index[0]][index[1]].setStatus((int) request.status);
  }
  result = true;

  response.result = result;

  if (result)
    ROS_INFO("Update Map Success!");
  else
    ROS_INFO("Update Map Fail!");


  for (int i = Common::rowCells - 1; i >= 0; i--) {
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
  }

  return true;
}

bool MapService::validMegaCells(is_kobuki::ValidMegaCells::Request& request,
                                is_kobuki::ValidMegaCells::Response& response) {
  ROS_INFO("Get valid megaCells");

  int robotId = (int) request.robotId;
  bool result = false;

  for (int i = 0; i < Common::rowCells / 2; i++)
    for (int j = 0; j < Common::colCells / 2; j++)
      if (Common::megaCells[i][j].getStatus() == WAITING &&
          !Common::megaCells[i][j].hasObstacle()) {
          response.rows.push_back(i);
          response.cols.push_back(j);
          result = true;
      }

  response.result = result;
  if (result)
    ROS_INFO("Has valid megaCells!");
  else
    ROS_INFO("No valid megaCells!");

  return true;
}

bool MapService::divideMap(is_kobuki::DivideMap::Request& request,
                             is_kobuki::DivideMap::Response& response) {

  ROS_INFO("Divide Map");

  int minRow = 0;
  int maxRow = Common::rowCells - 1;
  int minCol1 = 0;
  int maxCol1, minCol2;
  int maxCol2 = Common::colCells - 1;
  if((Common::colCells / 2) % 2 == 0 ) {
    maxCol1 = Common::colCells / 2 - 1;
    minCol2 = Common::colCells / 2;
  } else {
    maxCol1 = Common::colCells / 2;
    minCol2 = Common::colCells / 2 + 1;
  }

  int robotId = (int) request.robotId;
  if(robotId == 1) {
    robots[0] = Robot();
    robots[0].setStatus(NORMAL);
    response.minRow = minRow;
    response.maxRow = maxRow;
    response.minCol = minCol1;
    response.maxCol = maxCol1;
  } else if(robotId == 2){
    robots[1] = Robot();
    robots[1].setStatus(NORMAL);
    response.minRow = minRow;
    response.maxRow = maxRow;
    response.minCol = minCol2;
    response.maxCol = maxCol2;
  } else {
    ROS_INFO("ERROR robotId in function MapService::deviceMap");
  }

  ROS_INFO("Device Map Success!");
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
