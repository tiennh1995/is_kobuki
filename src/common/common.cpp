#include "../../include/common/common.hpp"

Cell **Common::cells;
MegaCell **Common::megaCells;
int Common::rowCells = 0;
int Common::colCells = 0;

Cell Common::findCell(int x, int y) {
	for (int i = 0; i < Common::rowCells; i++) {
		for (int j = 0; j < Common::colCells; j++)
			if (Common::cells[i][j].getX() == x && y == Common::cells[i][j].getY())
				return Common::cells[i][j];
	}
	return Cell();
}

// Cell Common::findCell(int x, int y) {
// 	int minX = 0;
// 	int maxX = Common::colCells - 1;
// 	int minY = 0;
// 	int maxY = Common::rowCells - 1;

// 	return findCellWithPosition(x, y, minX, maxX, minY, maxY);
// }

Cell Common::findCellWithPosition(int x, int y, int minX, int maxX, int minY, int maxY) {
	if (maxY < minY || maxX < minX)
		return Cell();

	int middleX = (minX + maxX) / 2;
	int middleY = (minY + maxY) / 2;
	if (Common::cells[middleX][middleY].getX() == x && Common::cells[middleX][middleY].getY() == y)
		return Common::cells[middleX][middleY];
	else if (y > Common::cells[middleX][middleY].getY())
		return findCellWithPosition(x, y, minX, maxX, middleY + 1, maxY);
	else if (y < Common::cells[middleX][middleY].getY())
		return findCellWithPosition(x, y, minX, maxX, minY, middleY - 1);
	else if (x > Common::cells[middleX][middleY].getX())
		return findCellWithPosition(x, y, middleX + 1, maxX, minY, maxY);
	else
		return findCellWithPosition(x, y, minX, middleX - 1, minY, maxY);
}

MegaCell Common::findMegaCell(int x, int y) {
	for (int i = 0; i < Common::rowCells / 2; i++) {
		for (int j = 0; j < Common::colCells / 2; j++)
			if (Common::megaCells[i][j].getX() == x && y == Common::megaCells[i][j].getY())
				return Common::megaCells[i][j];
	}
	return MegaCell();
}

// MegaCell Common::findMegaCell(int x, int y) {
// 	int minX = 0;
// 	int maxX = Common::colCells / 2;
// 	int minY = 0;
// 	int maxY = Common::rowCells / 2;

// 	return findMegaCellWithPosition(x, y, minX, maxX, minY, maxY);
// }

MegaCell Common::findMegaCellWithPosition(int x, int y, int minX, int maxX, int minY, int maxY) {
	if (maxY < minY || maxX < minX)
		return MegaCell();

	int middleX = (minX + maxX) / 2;
	int middleY = (minY + maxY) / 2;
	if (Common::megaCells[middleX][middleY].getX() == x && Common::megaCells[middleX][middleY].getY() == y)
		return Common::megaCells[middleX][middleY];
	else if (y > Common::megaCells[middleX][middleY].getY())
		return findMegaCellWithPosition(x, y, minX, maxX, middleY + 1, maxY);
	else if (y < Common::megaCells[middleX][middleY].getY())
		return findMegaCellWithPosition(x, y, minX, maxX, minY, middleY - 1);
	else if (x > Common::megaCells[middleX][middleY].getX())
		return findMegaCellWithPosition(x, y, middleX + 1, maxX, minY, maxY);
	else
		return findMegaCellWithPosition(x, y, minX, middleX - 1, minY, maxY);
}


MegaCell Common::findMegaCellByCell(Cell cell) {
	for (int i = 0; i < Common::rowCells / 2; i++) {
		for (int j = 0; j < Common::colCells / 2; j++)
			if (Common::megaCells[i][j].getCellPosition(cell) != -1)
				return Common::megaCells[i][j];
	}
	return MegaCell();
}

int* Common::findIndexCell(Cell cell) {
	int* index = (int*)malloc(2 * sizeof(int));
	for (int i = 0; i < Common::rowCells; i++) {
		for (int j = 0; j < Common::colCells; j++)
			if (Common::cells[i][j].getX() == cell.getX() &&
			    Common::cells[i][j].getY() == cell.getY()) {
				index[0] = i;
				index[1] = j;
				return index;
			}
	}

	index[0] = -1;
	index[1] = -1;
	return index;
}
