#ifndef STC_NAVIGATION_H
#define STC_NAVIGATION_H

#include <queue>
#include <stack>
#include <vector>

#include "../common/common.hpp"

class STCNavigation {
public:
  Cell startCell;
  MegaCell startMegaCell;

  Cell currentCell;
  MegaCell currentMegaCell;

  Cell parentCell;
  MegaCell parentMegaCell;

  int currentDirection;

  // Luu toan bo cac cell da di qua, dung cho viec sinh duong di
  std::queue<Cell> passedCellPath;
  std::queue<int> passedCellDirection;

  // Luu toan bo cac Megacell da di qua, dung de cho viec sinh duong di
  std::stack<MegaCell> passedMegaCellPath;

  // Chua nhung MegaCell da di qua.
  std::vector<MegaCell> passedMegaCell;

  // Constructor
  STCNavigation() {
    passedCellPath = std::queue<Cell>();
    passedCellDirection = std::queue<int>();
    passedMegaCellPath = std::stack<MegaCell>();
    passedMegaCell = std::vector<MegaCell>();
    currentDirection = D_DOWN;
  }

  // Destructor
  ~STCNavigation() {}

  // Kiem tra megaCell da di qua chua?
  bool checkPassedMegaCell(MegaCell megaCell);

  // Kiem tra megaCell co the di vao duoc k?
  bool validMegaCell(MegaCell megaCell);

  // Lay huong cua robot. (Kiem tra huong giua parentMegaCell va currentMegaCell)
  int getDirection();

  // Scan valid neighbor
  MegaCell scanNeighbor(int isInitialize);

  // Generate path from this cell to megaCell
  void generatePath(MegaCell megaCell);

  int changeCurrentDirection(int direction);

  // Xoa passedCellPath, passedMegaCellPath
  void clearPath();

  // Xoa passedCellPath
  void clearPassedCellPath();
};

#endif
