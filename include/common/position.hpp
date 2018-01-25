#ifndef POSITION_H
#define POSITION_H

#include <stdio.h>
#include <ros/console.h>

// Vi tri cua cell trong megaCell
enum CellPosition {
  //
  //                           |
  //            (1)TOPLEFT  ___|___ TOPRIGHT(4)
  //                       |   |   |
  //                  _____|___|___|
  //                       |   |   |
  //                       |___|___|
  //          (2)BOTTOMLEFT    |  BOTTOMRIGHT(3)
  //                           |
  //
  TOP_LEFT,
  BOTTOM_LEFT,
  BOTTOM_RIGHT,
  TOP_RIGHT
};

// Vi tri tuong doi giua cac megaCell
enum MegaCellPosition {
  //
  /*              Up(1)
   *               x
   *      Left(2) xxx Right(4)
   *               x
   *             Down(3)
   * */
  UP,
  LEFT,
  DOWN,
  RIGHT
};

#endif
