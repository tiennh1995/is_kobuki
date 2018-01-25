#ifndef MAP_H
#define MAP_H

#include <stdio.h>
#include <ros/console.h>

class Map {
private:
  int height;
  int width;
  double resolution;
  double originX;
  double originY;
  int **mapData;


public:
  Map() {
    height = 0;
    width = 0;
    resolution = 0;
    originX = 0;
    originY = 0;
    mapData = NULL;
  }

  Map(int height, int width, double resolution, double originX,
      double originY, int **mapData) {
    this->height = height;
    this->width = width;
    this->resolution = resolution;
    this->originX = originX;
    this->originY = originY;
    this->mapData = mapData;
  }

  int getHeight();
  int getWidth();
  double getResolution();
  double getOriginX();
  double getOriginY();
  int **getMapData();

  void setHeight(int height);
  void setWidth(int width);
  void setResolution(double resolution);
  void setOriginX(double originX);
  void setOriginY(double originY);
  void setMapData(int **mapData);

  bool isNULL();

  // Convert std:::vector<int> array to int[][] array
  int **oneArrToTwoArr(std::vector<int> mapData);

  // Tim kiem gioi han phia tren cua ban do (Dung cho viec thu gon ban do)
  int findLineUp();
  int findLineDown();
  int findLineLeft();
  int findLineRight();

  // Kiem tra 1 cell co vat can hay k. (Dang trong qua trinh khoi tao cell)
  bool checkObstacle(int x, int y, int cellSize);
};

#endif
