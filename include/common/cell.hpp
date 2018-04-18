#ifndef CELL_H
#define CELL_H

#include "position.hpp"

// Tren ban do toa do [0,0] nam o goc duoi ben trai (BOTTOM_LEFT)
class Cell {
private:
  //  column, row top_left position of Cell
  int x, y;

  // column, row
  int centreX, centreY;

  // size of a Cell
  int cellSize;

  // Co vat can hay k?
  bool obstacle;

  // Da quet hay chua
  int status;

public:
  Cell() {
    x = 0;
    y = 0;
    centreX = 0;
    centreY = 0;
    cellSize = 0;
    obstacle = true;
    status = WAITING;
  }

  Cell(int x, int y, int cellSize, bool obstacle) {
    this->x = x;
    this->y = y;
    this->cellSize = cellSize;
    this->obstacle = obstacle;

    // set centreX, centreY
    centreX = x + cellSize / 2;
    centreY = y - cellSize / 2;

    status = WAITING;
  }

  int getX();
  int getY();
  int getCentreX();
  int getCentreY();
  int getCellSize();
  bool hasObstacle();
  int getStatus();

  void setX(int x);
  void setY(int y);
  void setCellSize(int cellSize);
  void setCentreX(int centreX);
  void setCentreY(int centreY);
  void setObstacle(bool obstacle);
  void setStatus(int status);

  // Tra ve cell ben canh theo vi tri position
  Cell getNeighbor(int position);

  // Tra ve vi tri tuong doi cua tham so cell so voi this
  int getNeighbor(Cell cell);

  // Kiem tra cell null
  bool isNULL();

  // So sanh cell == other
  bool operator==(const Cell &other);

  // So sanh cell != other
  bool operator!=(const Cell &other);
};

#endif /* CELL_H */
