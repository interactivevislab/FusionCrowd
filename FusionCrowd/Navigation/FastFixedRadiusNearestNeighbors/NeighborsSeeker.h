#pragma once

#include "GpuCalculator.h"
#include "Config.h"

namespace FusionCrowd
{
	class Point;

	class FUSION_CROWD_API NeighborsSeeker
	{
	private:
		struct FUSION_CROWD_API GridCell {
			int pointsCount = 0;
			int startIndex = 0;
		};
		struct PointNeighbors;

		Point* points = nullptr;
		GridCell* cells = nullptr;
		int* pointsCells = nullptr;
		int* pointsInCells = nullptr;
		int* sortedPointsId = nullptr;
		PointNeighbors* pointNeighbors = nullptr;

		int numberOfPoints;
		int numberOfCells;
		static const int NUMBER_OF_NEIGHBORS = 10;
		float searchRadius;
		int maxNearCells;
		float cellSize;
		int cellsInRow;
		int cellsInColumn;

		GpuCalculator _calculator;
		InputBufferDesc _bufferDescriptions[3];
		bool _isCalculatorReady = false;

		void FillCellDictioanary();
		void CountIndeces();
		void SortPoints();

		int GetCellOffset(float coordinateOffset);
		int GetCellIndex(int cellX, float cellY);
		int GetCellIndex(float xCoord, float yCoord);
		bool IsInsideCell(float pointX, float pointY, int cellX, int cellY);
		bool IsCircleCrossesCell(float circleX, float circleY, int cellX, int cellY);

		void AllocateMemory();
		void FreeMemory();
		void ClearOldData(bool useGpu);
		void FindNeighborsCpu();
		void FindNeighborsGpu();
		void PrepareGpuCalculator();

		struct FUSION_CROWD_API CpuConstants {
			float cellSize;
			int cellsInRow;
			int cellsInColumn;
			float searchRadius;
		};

	public:
		NeighborsSeeker();
		~NeighborsSeeker();

		static float gridCellCoeff;

		struct FUSION_CROWD_API PointNeighbors {
			int pointID;
			int neighborsCount = 0;
			int neighborsID[NUMBER_OF_NEIGHBORS];
		};

		void Init(Point* points, int numberOfPoints, float worldWidth, float worldHeight, float searchRadius);
		PointNeighbors* FindNeighbors(bool useGpu = true);
	};


	class FUSION_CROWD_API Point {
	public:
		float x;
		float y;
		bool InRange(Point otherPoint, float range);
	};

}