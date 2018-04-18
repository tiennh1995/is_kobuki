#include "../../include/common/common.hpp"

int Robot::getId() {
  return id;
}

int Robot::getMinRow() {
  return minRow;
}

int Robot::getMaxRow() {
  return maxRow;
}

int Robot::getMinCol() {
  return minCol;
}

int Robot::getMaxCol() {
  return maxCol;
}

bool Robot::getStatus() {
  return status;
}

void Robot::setId(int id) {
  this->id = id;
}

void Robot::setMinRow(int minRow) {
  this->minRow = minRow;
}

void Robot::setMaxRow(int maxRow) {
  this->maxRow = maxRow;
}

void Robot::setMinCol(int minCol) {
  this->minCol = minCol;
}

void Robot::setMaxCol(int maxCol) {
  this->maxCol = maxCol;
}

void Robot::setStatus(bool status) {
  this->status = status;
}
