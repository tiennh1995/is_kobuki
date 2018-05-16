#ifndef ROBOT_H
#define ROBOT_H

#include <stdio.h>
#include <ros/console.h>

class Robot {
private:
  int id;
  int status;
  double updateTime;

public:
  Robot() {
    id = 0;
    status = true;
    updateTime = 0;
  }

  int getId();
  int getStatus();
  double getUpdateTime();

  void setId(int id);
  void setStatus(int status);
  void setUpdateTime();
};

#endif
