#include "NeighborsSeeker.h"
#include "GpuCalculator.h"
#include <iostream>
#include <string>
#include "Math/Util.h"

namespace FusionCrowd
{

	namespace {
		bool InRange(float x1, float y1, float x2, float y2, float range) {
			return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) < range * range;
		}
	}

	const std::string NeighborsSeeker::NUMBER_OF_NEIGHBORS_AS_STRING = std::to_string(NUMBER_OF_NEIGHBORS);

	NeighborsSeeker::NeighborsSeeker()
	{
		_calculator.Init();

		const D3D_SHADER_MACRO defines[] =
		{
			{ "MAX_NEIGHBORS", NUMBER_OF_NEIGHBORS_AS_STRING.c_str() },
			{ nullptr, nullptr }
		};

		bool isShaderLoaded = _calculator.LoadShader(L"NeighborsSeeker.hlsl", "main", defines);
		if (!isShaderLoaded) {
			throw std::runtime_error("An error occurred during shader loading.");
		}
	}


	NeighborsSeeker::~NeighborsSeeker()
	{
		FreeMemory();
	}


	float NeighborsSeeker::gridCellCoeff = 1.f;


	void NeighborsSeeker::PrepareMemory() {
		if (lastNumberOfPoints < numberOfPoints)
		{
			lastNumberOfPoints = max(3 * lastNumberOfPoints / 2, numberOfPoints);
			delete[] sortedPointsId;
			sortedPointsId = new int[lastNumberOfPoints];
			delete[] pointsCells;
			pointsCells = new int[lastNumberOfPoints];
			delete[] pointNeighbors;
			pointNeighbors = new PointNeighbors[lastNumberOfPoints];
		}

		if (lastNumberOfCells < numberOfCells)
		{
			lastNumberOfCells = max(3 * lastNumberOfCells / 2, numberOfCells);
			delete[] cells;
			cells = new GridCell[lastNumberOfCells];
			delete[] pointsInCells;
			pointsInCells = new int[lastNumberOfCells];
		}
	}


	void NeighborsSeeker::FreeMemory() {
		delete[] cells;
		delete[] sortedPointsId;
		delete[] pointsCells;
		delete[] pointsInCells;
		delete[] pointNeighbors;
		cells = nullptr;
		sortedPointsId = nullptr;
		pointsCells = nullptr;
		pointsInCells = nullptr;
		pointNeighbors = nullptr;
	}


	void NeighborsSeeker::SetInitialData(bool useGpu) {
		cells[0].startIndex = 0;
		for (int i = 0; i < numberOfCells; i++) {
			pointsInCells[i] = 0;
			cells[i].pointsCount = 0;
		}
		if (!useGpu) {
			if (pointNeighbors == nullptr) {
				pointNeighbors = new PointNeighbors[numberOfPoints];
				for (int i = 0; i < numberOfPoints; i++) {
					pointNeighbors[i].pointID = i;
				}
			}
			for (int i = 0; i < numberOfPoints; i++) {
				pointNeighbors[i].neighborsCount = 0;
			}
		}
	}


	void NeighborsSeeker::Init(Point* points, int numberOfPoints, float worldWidth, float worldHeight, float searchRadius) {
		this->points = points;
		this->numberOfPoints = numberOfPoints;

		this->searchRadius = searchRadius;
		cellSize = 2.f * searchRadius / gridCellCoeff;
		cellsInRow = ceil(worldWidth / cellSize);
		cellsInColumn = ceil(worldHeight / cellSize);
		numberOfCells = cellsInRow * cellsInColumn;
		float cellsInRadius = ceil(gridCellCoeff) + 1;
		maxNearCells = cellsInRadius * cellsInRadius;

		PrepareMemory();
	}


	void NeighborsSeeker::FindNeighborsCpu() {
		for (int i = 0; i < numberOfPoints; i++) {
			Point point = points[i]; // i = id
			int startCellX = GetCellOffset(point.x - searchRadius);
			int startCellY = GetCellOffset(point.y - searchRadius);
			int endCellX = GetCellOffset(point.x + searchRadius);
			int endCellY = GetCellOffset(point.y + searchRadius);

			if (startCellX < 0) startCellX = 0;
			if (startCellY < 0) startCellY = 0;
			if (endCellX >= cellsInRow) endCellX = cellsInRow - 1;
			if (endCellY >= cellsInColumn) endCellY = cellsInColumn - 1;

			int* nearCellsIndeces = new int[maxNearCells];
			int numberOfNearCells = 0;
			for (int cellY = startCellY; cellY <= endCellY; cellY++) {
				for (int cellX = startCellX; cellX <= endCellX; cellX++) {
					if (IsCircleCrossesCell(point.x, point.y, cellX, cellY)) {
						nearCellsIndeces[numberOfNearCells] = GetCellIndex(cellX, cellY);
						numberOfNearCells++;
					}
				}
			}

			bool enoughNeighbors = false;
			for (int cellIndexIndex = 0; cellIndexIndex < numberOfNearCells; cellIndexIndex++) {
				GridCell cell = cells[nearCellsIndeces[cellIndexIndex]];
				for (int pointIndex = cell.startIndex; pointIndex < cell.startIndex + cell.pointsCount; pointIndex++) {
					int otherPointIndex = sortedPointsId[pointIndex];
					if (i == otherPointIndex) continue;
					Point otherPoint = points[otherPointIndex];
					if (point.InRange(otherPoint, searchRadius)) {
						PointNeighbors* neighbors = &pointNeighbors[i];
						neighbors->neighborsID[neighbors->neighborsCount] = otherPointIndex;
						neighbors->neighborsCount++;
						if (neighbors->neighborsCount == NUMBER_OF_NEIGHBORS) {
							enoughNeighbors = true;
							break;
						}
					}
				}
				if (enoughNeighbors) break;
			}

			delete[] nearCellsIndeces;
		}
	}


	void NeighborsSeeker::FindNeighborsGpu() {
		_calculator.FreeUnusedMemory();

		_bufferDescriptions[0] = { sizeof(Point), numberOfPoints, points };
		_bufferDescriptions[1] = { sizeof(GridCell), numberOfCells, cells };
		_bufferDescriptions[2] = { sizeof(int), numberOfPoints, sortedPointsId };

		CpuConstants constants = { cellSize , cellsInRow , cellsInColumn , searchRadius };

		_calculator.SetInputBuffers(3, _bufferDescriptions);
		_calculator.SetConstantBuffer(sizeof(CpuConstants), 1, &constants);
		_calculator.SetOutputBuffer(sizeof(PointNeighbors), numberOfPoints);

		_calculator.RunShader();
		_calculator.GetResult(pointNeighbors);
	}


	NeighborsSeeker::PointNeighbors* NeighborsSeeker::FindNeighbors(Point* points, Math::Geometry2D** shapes, int numberOfPoints) {
		this->points = points;
		this->shapes = shapes;
		this->numberOfPoints = numberOfPoints;
		FindNeighborsCpuSquare();
		return pointNeighbors;

	}

	NeighborsSeeker::PointNeighbors* NeighborsSeeker::FindNeighbors(Point* points, int numberOfPoints,
		float worldWidth, float worldHeight, float searchRadius, bool useGpu)
	{
		Init(points, numberOfPoints, worldWidth, worldHeight, searchRadius);
		SetInitialData(useGpu);

		FillCellDictioanary();
		CountIndeces();
		SortPoints();

		useGpu ? FindNeighborsGpu() : FindNeighborsCpu();

		return pointNeighbors;
	}

#pragma region CpuLogic

	int NeighborsSeeker::GetCellOffset(float coordinateOffset) {
		return (int)floor(coordinateOffset / cellSize);
	}


	int NeighborsSeeker::GetCellIndex(int cellX, float cellY) {
		return cellX + cellsInRow * cellY;
	}


	int NeighborsSeeker::GetCellIndex(float xCoord, float yCoord) {
		return GetCellIndex(GetCellOffset(xCoord), GetCellOffset(yCoord));
	}


	bool NeighborsSeeker::IsInsideCell(float pointX, float pointY, int cellX, int cellY) {
		float cellLeftBorder = cellX * cellSize;
		float cellUpperBorder = cellY * cellSize;
		return (pointX > cellLeftBorder) && (pointX < cellLeftBorder + cellSize)
			&& (pointY > cellUpperBorder) && (pointY < cellUpperBorder + cellSize);
	}


	bool NeighborsSeeker::IsCircleCrossesCell(float circleX, float circleY, int cellX, int cellY) {
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


	void NeighborsSeeker::FillCellDictioanary() {
		int cellIndex;
		for (int i = 0; i < numberOfPoints; i++) {
			cellIndex = GetCellIndex(points[i].x, points[i].y);
			pointsCells[i] = cellIndex;
			cells[cellIndex].pointsCount++;
		}
	}


	void NeighborsSeeker::CountIndeces() {
		for (int i = 1; i < numberOfCells; i++) {
			cells[i].startIndex = cells[i - 1].startIndex + cells[i - 1].pointsCount;
		}
	}


	void NeighborsSeeker::SortPoints() {
		int cellIndex;
		int pointIndex;
		for (int i = 0; i < numberOfPoints; i++) {
			cellIndex = pointsCells[i];
			pointIndex = cells[cellIndex].startIndex + pointsInCells[cellIndex];
			sortedPointsId[pointIndex] = i;
			pointsInCells[cellIndex]++;
		}
	}


	bool Point::InRange(Point otherPoint, float range) {
		return FusionCrowd::InRange(x, y, otherPoint.x, otherPoint.y, range);
	}

	void NeighborsSeeker::FindNeighborsCpuSquare() {
		using namespace DirectX::SimpleMath;
		if (pointNeighbors == nullptr) {
			pointNeighbors = new PointNeighbors[numberOfPoints];
			for (int i = 0; i < numberOfPoints; i++) {
				pointNeighbors[i].pointID = i;
			}
		}
		for (int i = 0; i < numberOfPoints; i++) {
			pointNeighbors[i].neighborsCount = 0;
		}


		for (int i = 0; i < numberOfPoints; i++) {
			Math::Geometry2D& geometry = *shapes[i]; // i = id
			auto& point = points[i];
			for (int j = 0; j < numberOfPoints; j++) {
				if (i == j) continue;
				Point otherPoint = points[j];
				if (geometry.containsPoint(Vector2(otherPoint.x, otherPoint.y), Vector2(point.x, point.y), point.rot)) {
					PointNeighbors* neighbors = &pointNeighbors[i];
					neighbors->neighborsID[neighbors->neighborsCount] = j;
					neighbors->neighborsCount++;
					if (neighbors->neighborsCount == NUMBER_OF_NEIGHBORS) {
						break;
					}
				}
			}
		}
	}
#pragma endregion

}