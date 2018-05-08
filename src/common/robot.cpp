#include "../../include/common/common.hpp"

int Robot::getId() {
  return id;
}

double Robot::getStartTime() {
  return startTime;
}

double Robot::getFinishTime() {
  return finishTime;
}

int Robot::getStatus() {
  return status;
}

void Robot::setId(int id) {
  this->id = id;
}

void Robot::setStartTime() {
  this->startTime = ros::Time::now().toSec();
}

void Robot::setFinishTime() {
  this->finishTime = ros::Time::now().toSec();
}

void Robot::setStatus(int status) {
  this->status = status;
}
