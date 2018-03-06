#ifndef COMMON_H
#define COMMON_H

#include "status.hpp"
#include "cell.hpp"
#include "mega_cell.hpp"
#include "direction.hpp"
#include "map.hpp"
#include "robot.hpp"

class Common {
private:
  static Cell findCellWithPosition(int x, int y, int minX, int maxX, int minY, int maxY);
  static MegaCell findMegaCellWithPosition(int x, int y, int minX, int maxX, int minY, int maxY);

public:
  static Cell **cells;
  static MegaCell **megaCells;

  static int rowCells;
  static int colCells;

  static Cell findCell(int x, int y);

  static MegaCell findMegaCell(int x, int y);

  static MegaCell findMegaCellByCell(Cell cell);

  static int* findIndexCell(Cell cell);
};

#endif
