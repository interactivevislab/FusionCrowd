#pragma once

#include "GpuCalculator.h"
#include "Navigation/AgentSpatialInfo.h"
#include "Math/Geometry2D.h"

#include <vector>
#include <unordered_map>

namespace FusionCrowd
{
	class NeighborsSeeker
	{
	public:
		using SearchRequest = AgentSpatialInfo;
		using SearchResult = std::unordered_map<size_t, std::vector<size_t>>;

	public:
		NeighborsSeeker();

		SearchResult FindNeighborsCpu(std::vector<SearchRequest> searchRequests);
	};
}
