#ifndef MEGA_CELL_H
#define MEGA_CELL_H

#include "position.hpp"
#include "cell.hpp"

// Tren ban do toa do 0,0 nam o goc duoi ben trai (BOTTOM_LEFT)

// Vi tri cua megaCell duoc dinh nghia la TOP_LEFT
class MegaCell {
private:
  int x, y;
  int cellSize;
  bool obstacle;

public:
  MegaCell() {
    x = 0;
    y = 0;
    cellSize = 0;
    obstacle = true;
  }

  MegaCell(Cell cell, int position, bool obstacle) {
    cellSize = cell.getCellSize();

    if (position == TOP_LEFT) {
      x = cell.getX();
      y = cell.getY();
    }

    if (position == TOP_RIGHT) {
      x = cell.getX() - cell.getCellSize();
      y = cell.getY();
    }

    if (position == BOTTOM_LEFT) {
      x = cell.getX();
      y = cell.getY() + cell.getCellSize();
    }

    if (position == BOTTOM_RIGHT) {
      x = cell.getX() - cell.getCellSize();
      y = cell.getY() + cell.getCellSize();
    }

    this->obstacle = obstacle;
  }

  int getX();
  int getY();
  int getCellSize();
  bool hasObstacle();

  void setX(int x);
  void setY(int y);
  void setCellSize(int cellSize);
  void setObstacle(bool obstacle);

  // Tra ve cell trong megaCell theo position
  Cell getCell(int position);

  // Tra ve vi tri position cua cell
  int getCellPosition(Cell cell);

  // Tra ve vi tri tuong doi cua megaCell so voi this
  int getMegaCellPosition(MegaCell megaCell);

  // Lay megaCell xung quanh, theo chieu nguoc chieu kim dong ho.
  MegaCell getNeighbor(int position);

  // So sanh megaCell == other
  bool operator==(const MegaCell &other);

  // So sanh megaCell != other
  bool operator!=(const MegaCell &other);

  // Kiem tra megaCell null
  bool isNULL();
};

#endif
