#include "../../include/common/common.hpp"

int Map::getHeight() {
  return height;
}

int Map::getWidth() {
  return width;
}

double Map::getResolution() {
  return resolution;
}

double Map::getOriginX() {
  return originX;
}

double Map::getOriginY() {
  return originY;
}

int **Map::getMapData() {
  return mapData;
}

void Map::setHeight(int height) {
  this->height = height;
}

void Map::setWidth(int width) {
  this->width = width;
}

void Map::setResolution(double resolution) {
  this->resolution = resolution;
}

void Map::setOriginX(double originX) {
  this->originX = originX;
}

void Map::setOriginY(double originY) {
  this->originY = originY;
}

void Map::setMapData(int **mapData) {
  this->mapData = mapData;
}

bool Map::isNULL() {
  return this->mapData == NULL;
}

// Chuyen mapdata tu 1 chieu sang 2 chieu
int** Map::oneArrToTwoArr(std::vector<int> mapData) {
  int **mapArr = (int **)malloc(getHeight() * sizeof(int *));
  for (int i = 0; i < getHeight(); i++)
    mapArr[i] = (int *)malloc(getWidth() * sizeof(int));

  int row = 0;  // hang
  int col = 0;  // cot

  for (int i = 0; i < mapData.size(); i++) {
    mapArr[row][col] = mapData[i];
    col++;
    if (col == getWidth()) {
      row++;
      col = 0;
    }
  }

  return mapArr;
}

int Map::findLineUp() {
  int row = 0;
  int col = 0;
  for (row = getHeight() - 1; row >= 0; row--) {
    for (col = 0; col < getWidth(); col++) {
      if (getMapData()[row][col] == 100 || getMapData()[row][col] == 0)
        return row;
    }
  }
  return 0;
}

int Map::findLineDown() {
  int row = 0, col = 0;
  for (row = 0; row < getHeight(); row++) {
    for (col = 0; col < getWidth(); col++) {
      if (getMapData()[row][col] == 100 || getMapData()[row][col] == 0)
        return row;
    }
  }
  return 0;
}

int Map::findLineLeft() {
  int row = 0, col = 0;
  for (col = 0; col < getWidth(); col++) {
    for (row = 0; row < getHeight(); row++) {
      if (getMapData()[row][col] == 100 || getMapData()[row][col] == 0)
        return col;
    }
  }
  return 0;
}

int Map::findLineRight() {
  int row = 0, col = 0;
  for (col = getWidth() - 1; col >= 0; col--) {
    for (row = 0; row < getHeight(); row++) {
      if (getMapData()[row][col] == 100 || getMapData()[row][col] == 0)
        return col;
    }
  }
  return 0;
}

bool Map::checkObstacle(int x, int y, int cellSize) {
  int row, col;
  for (row = y; row > y - cellSize; row--) {
    for (col = x; col < x + cellSize; col++) {
      if (getMapData()[row][col] != 0) {
        return true;
      }
    }
  }
  return false;
}
