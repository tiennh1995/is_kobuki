
#include "../../include/stc/stc_navigation.hpp"

bool STCNavigation::checkPassedMegaCell(MegaCell megaCell) {
  for (int i = 0; i < passedMegaCell.size(); i++) {
    if (passedMegaCell[i] == megaCell)
      return true;
  }
  return false;
}

bool STCNavigation::validMegaCell(MegaCell megaCell) {
  if (megaCell.hasObstacle() || megaCell.getStatus() == SCANED) {
    return false;
  }

  return !checkPassedMegaCell(megaCell);
}

int STCNavigation::getDirection() {
  if ((parentMegaCell.getX() == currentMegaCell.getX()) &&
      (parentMegaCell.getY() > currentMegaCell.getY()))
    return D_DOWN;

  if ((parentMegaCell.getX() < currentMegaCell.getX()) &&
      (parentMegaCell.getY() == currentMegaCell.getY()))
    return D_RIGHT;

  if ((parentMegaCell.getX() == currentMegaCell.getX()) &&
      (parentMegaCell.getY() < currentMegaCell.getY()))
    return D_UP;

  if ((parentMegaCell.getX() > currentMegaCell.getX()) &&
      (parentMegaCell.getY() == currentMegaCell.getY()))
    return D_LEFT;
}

// Xoa passedCellPath
void STCNavigation::clearPassedCellPath() {
  while (!passedCellPath.empty())
    passedCellPath.pop();
}

// Xoa passedCellPath, passedMegaCellPath
void STCNavigation::clearPath() {
  clearPassedCellPath();

  while (!passedMegaCellPath.empty())
    passedMegaCellPath.pop();
}

//  Return first availabe MegaCell;
MegaCell STCNavigation::scanNeighbor(int isInitialize) {
  MegaCell neighborLEFT = currentMegaCell.getNeighbor(LEFT);
  MegaCell neighborRIGHT = currentMegaCell.getNeighbor(RIGHT);
  MegaCell neighborUP = currentMegaCell.getNeighbor(UP);
  MegaCell neighborDOWN = currentMegaCell.getNeighbor(DOWN);

  // Khi robot quet lan dau tien, bau dau tu vi tri xuat phat
  if (isInitialize == 1) {
    if (validMegaCell(neighborLEFT))
      return neighborLEFT;

    if (validMegaCell(neighborDOWN))
      return neighborDOWN;

    if (validMegaCell(neighborRIGHT))
      return neighborRIGHT;

    if (validMegaCell(neighborUP))
      return neighborUP;
  } else {
    int direction = getDirection();

    // parent UP, current DOWN
    if (direction == D_DOWN) {
      if (validMegaCell(neighborLEFT)) return neighborLEFT;
      if (validMegaCell(neighborDOWN)) return neighborDOWN;
      if (validMegaCell(neighborRIGHT)) return neighborRIGHT;
    }

    // parent LEFT, current RIGHT
    if (direction == D_RIGHT) {
      if (validMegaCell(neighborDOWN)) return neighborDOWN;
      if (validMegaCell(neighborRIGHT)) return neighborRIGHT;
      if (validMegaCell(neighborUP)) return neighborUP;
    }

    // parent DOWN, current UP
    if (direction == D_UP) {
      if (validMegaCell(neighborRIGHT)) return neighborRIGHT;
      if (validMegaCell(neighborUP)) return neighborUP;
      if (validMegaCell(neighborLEFT)) return neighborLEFT;
    }

    // parent RIGHT, current LEFT
    if (direction == D_LEFT) {
      if (validMegaCell(neighborUP)) return neighborUP;
      if (validMegaCell(neighborLEFT)) return neighborLEFT;
      if (validMegaCell(neighborDOWN)) return neighborDOWN;
    }
  }

  return currentMegaCell;
}

// Generate path from currentMegaCell to megaCell into passedCellPath
void STCNavigation::generatePath(MegaCell megaCell) {
  clearPassedCellPath();

  int cellPosition = currentMegaCell.getCellPosition(currentCell);
  int megaCellPosition = currentMegaCell.getMegaCellPosition(megaCell);

  // megaCell UP, current DOWN
  if (megaCellPosition == UP) {
    switch (cellPosition) {
    case TOP_LEFT:
      passedCellPath.push(currentCell.getNeighbor(DOWN));
      passedCellDirection.push(changeCurrentDirection(DOWN));
      passedCellPath.push(currentCell.getNeighbor(DOWN).getNeighbor(RIGHT));
      passedCellDirection.push(changeCurrentDirection(RIGHT));
      passedCellPath.push(currentCell.getNeighbor(DOWN).getNeighbor(RIGHT).getNeighbor(UP));
      passedCellDirection.push(changeCurrentDirection(UP));
      passedCellPath.push(currentCell.getNeighbor(DOWN).getNeighbor(RIGHT).getNeighbor(UP).getNeighbor(UP));
      passedCellDirection.push(changeCurrentDirection(UP));
      break;
    case TOP_RIGHT:
      passedCellPath.push(currentCell.getNeighbor(UP));
      passedCellDirection.push(changeCurrentDirection(UP));
      break;
    case BOTTOM_LEFT:
      passedCellPath.push(currentCell.getNeighbor(RIGHT));
      passedCellDirection.push(changeCurrentDirection(RIGHT));
      passedCellPath.push(currentCell.getNeighbor(RIGHT).getNeighbor(UP));
      passedCellDirection.push(changeCurrentDirection(UP));
      passedCellPath.push(currentCell.getNeighbor(RIGHT).getNeighbor(UP).getNeighbor(UP));
      passedCellDirection.push(changeCurrentDirection(UP));
      break;
    case BOTTOM_RIGHT:
      passedCellPath.push(currentCell.getNeighbor(UP));
      passedCellDirection.push(changeCurrentDirection(UP));
      passedCellPath.push(currentCell.getNeighbor(UP).getNeighbor(UP));
      passedCellDirection.push(changeCurrentDirection(UP));
      break;
    }
  }

  // megaCell LEFT, current RIGHT
  if (megaCellPosition == LEFT) {
    switch (cellPosition) {
    case TOP_LEFT:
      passedCellPath.push(currentCell.getNeighbor(LEFT));
      passedCellDirection.push(changeCurrentDirection(LEFT));
      break;
    case BOTTOM_LEFT:
      passedCellPath.push(currentCell.getNeighbor(RIGHT));
      passedCellDirection.push(changeCurrentDirection(RIGHT));
      passedCellPath.push(currentCell.getNeighbor(RIGHT).getNeighbor(UP));
      passedCellDirection.push(changeCurrentDirection(UP));
      passedCellPath.push(currentCell.getNeighbor(RIGHT).getNeighbor(UP).getNeighbor(LEFT));
      passedCellDirection.push(changeCurrentDirection(LEFT));
      passedCellPath.push(currentCell.getNeighbor(RIGHT).getNeighbor(UP).getNeighbor(LEFT).getNeighbor(LEFT));
      passedCellDirection.push(changeCurrentDirection(LEFT));
      break;
    case BOTTOM_RIGHT:
      passedCellPath.push(currentCell.getNeighbor(UP));
      passedCellDirection.push(changeCurrentDirection(UP));
      passedCellPath.push(currentCell.getNeighbor(UP).getNeighbor(LEFT));
      passedCellDirection.push(changeCurrentDirection(LEFT));
      passedCellPath.push(currentCell.getNeighbor(UP).getNeighbor(LEFT).getNeighbor(LEFT));
      passedCellDirection.push(changeCurrentDirection(LEFT));
      break;
    case TOP_RIGHT:
      passedCellPath.push(currentCell.getNeighbor(LEFT));
      passedCellDirection.push(changeCurrentDirection(LEFT));
      passedCellPath.push(currentCell.getNeighbor(LEFT).getNeighbor(LEFT));
      passedCellDirection.push(changeCurrentDirection(LEFT));
      break;
    }
  }

  // megaCell DOWN, current UP
  if (megaCellPosition == DOWN) {
    switch (cellPosition) {
    case BOTTOM_RIGHT:
      passedCellPath.push(currentCell.getNeighbor(UP));
      passedCellDirection.push(changeCurrentDirection(UP));
      passedCellPath.push(currentCell.getNeighbor(UP).getNeighbor(LEFT));
      passedCellDirection.push(changeCurrentDirection(LEFT));
      passedCellPath.push(currentCell.getNeighbor(UP).getNeighbor(LEFT).getNeighbor(DOWN));
      passedCellDirection.push(changeCurrentDirection(DOWN));
      passedCellPath.push(currentCell.getNeighbor(UP).getNeighbor(LEFT).getNeighbor(DOWN).getNeighbor(DOWN));
      passedCellDirection.push(changeCurrentDirection(DOWN));
      break;
    case BOTTOM_LEFT:
      passedCellPath.push(currentCell.getNeighbor(DOWN));
      passedCellDirection.push(changeCurrentDirection(DOWN));
      break;
    case TOP_RIGHT:
      passedCellPath.push(currentCell.getNeighbor(LEFT));
      passedCellDirection.push(changeCurrentDirection(LEFT));
      passedCellPath.push(currentCell.getNeighbor(LEFT).getNeighbor(DOWN));
      passedCellDirection.push(changeCurrentDirection(DOWN));
      passedCellPath.push(currentCell.getNeighbor(LEFT).getNeighbor(DOWN).getNeighbor(DOWN));
      passedCellDirection.push(changeCurrentDirection(DOWN));
      break;
    case TOP_LEFT:
      passedCellPath.push(currentCell.getNeighbor(DOWN));
      passedCellDirection.push(changeCurrentDirection(DOWN));
      passedCellPath.push(currentCell.getNeighbor(DOWN).getNeighbor(DOWN));
      passedCellDirection.push(changeCurrentDirection(DOWN));
      break;
    }
  }

  // megaCell RIGHT, current LEFT
  if (megaCellPosition == RIGHT) {
    switch (cellPosition) {
    case TOP_RIGHT:
      passedCellPath.push(currentCell.getNeighbor(LEFT));
      passedCellDirection.push(changeCurrentDirection(LEFT));
      passedCellPath.push(currentCell.getNeighbor(LEFT).getNeighbor(DOWN));
      passedCellDirection.push(changeCurrentDirection(DOWN));
      passedCellPath.push(currentCell.getNeighbor(LEFT).getNeighbor(DOWN).getNeighbor(RIGHT));
      passedCellDirection.push(changeCurrentDirection(RIGHT));
      passedCellPath.push(currentCell.getNeighbor(LEFT).getNeighbor(DOWN).getNeighbor(RIGHT).getNeighbor(RIGHT));
      passedCellDirection.push(changeCurrentDirection(RIGHT));
      break;
    case BOTTOM_RIGHT:
      passedCellPath.push(currentCell.getNeighbor(RIGHT));
      passedCellDirection.push(changeCurrentDirection(RIGHT));
      break;
    case TOP_LEFT:
      passedCellPath.push(currentCell.getNeighbor(DOWN));
      passedCellDirection.push(changeCurrentDirection(DOWN));
      passedCellPath.push(currentCell.getNeighbor(DOWN).getNeighbor(RIGHT));
      passedCellDirection.push(changeCurrentDirection(RIGHT));
      passedCellPath.push(currentCell.getNeighbor(DOWN).getNeighbor(RIGHT).getNeighbor(RIGHT));
      passedCellDirection.push(changeCurrentDirection(RIGHT));
      break;
    case BOTTOM_LEFT:
      passedCellPath.push(currentCell.getNeighbor(RIGHT));
      passedCellDirection.push(changeCurrentDirection(RIGHT));
      passedCellPath.push(currentCell.getNeighbor(RIGHT).getNeighbor(RIGHT));
      passedCellDirection.push(changeCurrentDirection(RIGHT));
      break;
    }
  }
}

int STCNavigation::changeCurrentDirection(int direction) {
  switch (currentDirection) {
  case D_UP:
    if (direction == UP) {
      currentDirection = D_UP;
      return D_UP;
    }

    if (direction == DOWN) {
      currentDirection = D_DOWN;
      return D_DOWN;
    }

    if (direction == LEFT) {
      currentDirection = D_LEFT;
      return D_LEFT;
    }

    if (direction == RIGHT) {
      currentDirection = D_RIGHT;
      return D_RIGHT;
    }
  case D_LEFT:
    if (direction == UP) {
      currentDirection = D_UP;
      return D_RIGHT;
    }

    if (direction == DOWN) {
      currentDirection = D_DOWN;
      return D_LEFT;
    }

    if (direction == LEFT) {
      currentDirection = D_LEFT;
      return D_UP;
    }

    if (direction == RIGHT) {
      currentDirection = D_RIGHT;
      return D_DOWN;
    }
  case D_RIGHT:
    if (direction == UP) {
      currentDirection = D_UP;
      return D_LEFT;
    }

    if (direction == DOWN) {
      currentDirection = D_DOWN;
      return D_RIGHT;
    }

    if (direction == LEFT) {
      currentDirection = D_LEFT;
      return D_DOWN;
    }

    if (direction == RIGHT) {
      currentDirection = D_RIGHT;
      return D_UP;
    }
  case D_DOWN:
    if (direction == UP) {
      currentDirection = D_UP;
      return D_DOWN;
    }

    if (direction == RIGHT) {
      currentDirection = D_RIGHT;
      return D_LEFT;
    }

    if (direction == LEFT) {
      currentDirection = D_LEFT;
      return D_RIGHT;
    }

    if (direction == DOWN) {
      currentDirection = D_DOWN;
      return D_UP;
    }
  }
}
