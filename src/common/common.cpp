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

MegaCell Common::findMegaCell(int x, int y) {
	for (int i = 0; i < Common::rowCells / 2; i++) {
		for (int j = 0; j < Common::colCells / 2; j++)
			if (Common::megaCells[i][j].getX() == x && y == Common::megaCells[i][j].getY())
				return Common::megaCells[i][j];
	}
	return MegaCell();
}

MegaCell Common::findMegaCellByCell(Cell cell) {
	for (int i = 0; i < Common::rowCells / 2; i++) {
		for (int j = 0; j < Common::colCells / 2; j++)
			if (Common::megaCells[i][j].getCellPosition(cell) != -1)
				return Common::megaCells[i][j];
	}
	return MegaCell();
}
