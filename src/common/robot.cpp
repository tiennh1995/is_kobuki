#include "../../include/common/common.hpp"

int Robot::getId() {
  return id;
}

double Robot::getUpdateTime() {
  return updateTime;
}

int Robot::getStatus() {
  return status;
}

void Robot::setId(int id) {
  this->id = id;
}

void Robot::setUpdateTime() {
  this->updateTime = ros::Time::now().toSec();
}

void Robot::setStatus(int status) {
  this->status = status;
}
