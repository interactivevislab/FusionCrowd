#pragma once

#include "GpuCalculator.h"
#include "Export/Config.h"
#include <string>

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
		static const int NUMBER_OF_NEIGHBORS = 20;
		static const std::string NUMBER_OF_NEIGHBORS_AS_STRING;
		float searchRadius;
		int maxNearCells;
		float cellSize;
		int cellsInRow;
		int cellsInColumn;

		GpuCalculator _calculator;
		InputBufferDesc _bufferDescriptions[3];

		int lastNumberOfPoints = 0;
		int lastNumberOfCells = 0;

		void FillCellDictioanary();
		void CountIndeces();
		void SortPoints();

		inline int GetCellOffset(float coordinateOffset);
		inline int GetCellIndex(int cellX, float cellY);
		inline int GetCellIndex(float xCoord, float yCoord);
		inline bool IsInsideCell(float pointX, float pointY, int cellX, int cellY);
		inline bool IsCircleCrossesCell(float circleX, float circleY, int cellX, int cellY);

		void PrepareMemory();
		void FreeMemory();
		void SetInitialData(bool useGpu);
		void FindNeighborsCpu();
		void FindNeighborsCpuSquare();
		void FindNeighborsGpu();

		void Init(Point* points, int numberOfPoints, float worldWidth, float worldHeight, float searchRadius);

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

		PointNeighbors* FindNeighbors(Point* points, int numberOfPoints, float worldWidth, float worldHeight,
			float searchRadius, bool useGpu = true, bool simplified = true);
	};


	class FUSION_CROWD_API Point {
	public:
		float x;
		float y;
		bool InRange(Point otherPoint, float range);
	};

}