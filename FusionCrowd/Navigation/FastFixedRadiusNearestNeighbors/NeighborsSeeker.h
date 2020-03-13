#pragma once

#include "GpuCalculator.h"
#include "Export/Config.h"
#include "Navigation/AgentSpatialInfo.h"
#include "Math/Geometry2D.h"
#include <string>
#include <vector>

namespace FusionCrowd
{
	class Point;

	class FUSION_CROWD_API NeighborsSeeker
	{
	public:
		NeighborsSeeker();
		~NeighborsSeeker();

		struct PointNeighbors {
			int pointID;
			std::vector<int> neighborsID;
		};

		std::vector<PointNeighbors> FindNeighborsCpuSquare(std::vector<AgentSpatialInfo> positions, std::vector<Math::Geometry2D*> search_areas);
	};


	class Point {
	public:
		float x;
		float y;
		float rot;
	};

}