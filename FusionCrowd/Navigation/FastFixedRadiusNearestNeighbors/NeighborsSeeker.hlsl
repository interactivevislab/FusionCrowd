struct Point {
	float x;
	float y;
};

struct GridCell {
	int pointsCount;
	int startIndex;
};

struct PointNeighbors {
	int pointID;
	int neighborsCount;
	int neighborsID[MAX_NEIGHBORS];
};

cbuffer globals : register(b0) {
	float cellSize;
	int cellsInRow;
	int cellsInColumn;
	float searchRadius;
};

StructuredBuffer<Point> points : register(t0);
StructuredBuffer<GridCell> cells : register(t1);
StructuredBuffer<int> sortedPointsId : register(t2);
RWStructuredBuffer<PointNeighbors> pointNeighbors : register(u0);

bool IsCircleCrossesCell(float circleX, float circleY, int cellX, int cellY);
bool IsInsideCell(float pointX, float pointY, int cellX, int cellY);
bool InRange(float x1, float y1, float x2, float y2, float range);

[numthreads(1, 1, 1)]
void main(uint3 DTid : SV_DispatchThreadID)
{
	Point currentPoint = points[DTid.x]; // index = id
	int startCellX = (int)floor((currentPoint.x - searchRadius) / cellSize);
	int startCellY = (int)floor((currentPoint.y - searchRadius) / cellSize);
	int endCellX = (int)floor((currentPoint.x + searchRadius) / cellSize);
	int endCellY = (int)floor((currentPoint.y + searchRadius) / cellSize);

	if (startCellX < 0) startCellX = 0;
	if (startCellY < 0) startCellY = 0;
	if (endCellX >= cellsInRow) endCellX = cellsInRow - 1;
	if (endCellY >= cellsInColumn) endCellY = cellsInColumn - 1;

	int nearCellsIndeces[25];
	int numberOfNearCells = 0;
	for (int cellY = startCellY; cellY <= endCellY; cellY++) {
		for (int cellX = startCellX; cellX <= endCellX; cellX++) {
			if (IsCircleCrossesCell(currentPoint.x, currentPoint.y, cellX, cellY)) {
				nearCellsIndeces[numberOfNearCells] = cellX + cellsInRow * cellY;
				numberOfNearCells++;
			}
		}
	}

	int neighborsCount = 0;
	bool enoughNeighbors = false;
	for (int cellIndexIndex = 0; cellIndexIndex < numberOfNearCells; cellIndexIndex++) {
		GridCell cell = cells[nearCellsIndeces[cellIndexIndex]];
		for (int pointIndex = cell.startIndex; pointIndex < cell.startIndex + cell.pointsCount; pointIndex++) {
			int otherPointIndex = sortedPointsId[pointIndex];
			if (DTid.x == otherPointIndex) continue;
			Point otherPoint = points[otherPointIndex];
			if (InRange(currentPoint.x, currentPoint.y, otherPoint.x, otherPoint.y, searchRadius)) {
				pointNeighbors[DTid.x].neighborsID[neighborsCount] = otherPointIndex;
				neighborsCount++;
				if (neighborsCount == MAX_NEIGHBORS) {
					enoughNeighbors = true;
					break;
				}
			}
		}
		if (enoughNeighbors) break;
	}
	pointNeighbors[DTid.x].pointID = DTid.x;
	pointNeighbors[DTid.x].neighborsCount = neighborsCount;
}

bool IsCircleCrossesCell(float circleX, float circleY, int cellX, int cellY) {
	//center
	if (IsInsideCell(circleX, circleY, cellX, cellY)) return true;

	//edges
	if (IsInsideCell(circleX + searchRadius, circleY, cellX, cellY)) return true;
	if (IsInsideCell(circleX - searchRadius, circleY, cellX, cellY)) return true;
	if (IsInsideCell(circleX, circleY + searchRadius, cellX, cellY)) return true;
	if (IsInsideCell(circleX, circleY - searchRadius, cellX, cellY)) return true;

	//corners
	float cellLeftBorder = cellX * cellSize;
	float cellUpperBorder = cellY * cellSize;
	if (InRange(circleX, circleY, cellLeftBorder, cellUpperBorder, searchRadius)) return true;
	if (InRange(circleX, circleY, cellLeftBorder + cellSize, cellUpperBorder, searchRadius)) return true;
	if (InRange(circleX, circleY, cellLeftBorder, cellUpperBorder + cellSize, searchRadius)) return true;
	if (InRange(circleX, circleY, cellLeftBorder + cellSize, cellUpperBorder + cellSize, searchRadius)) return true;

	return false;
}

bool IsInsideCell(float pointX, float pointY, int cellX, int cellY) {
	float cellLeftBorder = cellX * cellSize;
	float cellUpperBorder = cellY * cellSize;
	return (pointX > cellLeftBorder) && (pointX < cellLeftBorder + cellSize)
		&& (pointY > cellUpperBorder) && (pointY < cellUpperBorder + cellSize);
}

bool InRange(float x1, float y1, float x2, float y2, float range) {
	return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) < range * range;
}