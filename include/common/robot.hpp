#ifndef ROBOT_H
#define ROBOT_H

#include <stdio.h>
#include <ros/console.h>

class Robot {
private:
  int id;
  int status;
  double startTime, finishTime;

public:
  Robot() {
    id = 0;
    status = true;
    startTime = 0;
    finishTime = 0;
  }

  int getId();
  int getStatus();
  double getStartTime();
  double getFinishTime();

  void setId(int id);
  void setStatus(int status);
  void setStartTime();
  void setFinishTime();
};

#endif
