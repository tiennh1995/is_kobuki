#ifndef ROBOT_H
#define ROBOT_H

#include <stdio.h>
#include <ros/console.h>

class Robot {
private:
  int id;
  int minRow;
  int maxRow;
  int minCol;
  int maxCol;
  bool status;

public:
  Robot() {
    id = 0;
    minRow = 0;
    maxRow = 0;
    minCol = 0;
    maxCol = 0;
    status = true;
  }

  Robot(int id, int minRow, int maxRow, int minCol, int maxCol, bool status) {
    this->id = id;
    this->minRow = minRow;
    this->maxRow = maxRow;
    this->minCol = minCol;
    this->maxCol = maxCol;
    this->status = status;
  }

  int getId();
  int getMinRow();
  int getMaxRow();
  int getMinCol();
  int getMaxCol();
  bool getStatus();

  void setId(int id);
  void setMinRow(int minRow);
  void setMaxRow(int maxRow);
  void setMinCol(int minCol);
  void setMaxCol(int maxCol);
  void setStatus(bool status);
};

#endif
