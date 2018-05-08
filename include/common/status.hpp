#ifndef STATUS_H
#define STATUS_H

#include <stdio.h>
#include <ros/console.h>

enum Status {
  WAITING,
  SCANED,
  MOVING
};

#endif
