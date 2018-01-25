#ifndef DIRECTION_H
#define DIRECTION_H

#include <stdio.h>
#include <ros/console.h>

// Huong cua robot, dua theo currentMegaCell va parentMegaCell
enum MegaCellDirection {
            //          (D_UP)
            //            ^
            //            |
            //(D_LEFT) <=   => (D_RIGHT)
            //            |
            //            v
            //         (D_DOWN)

  D_UP,
  D_LEFT,
  D_DOWN,
  D_RIGHT
};

#endif
