#pragma once

#include "GpuCalculator.h"
#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/NeighborInfo.h"

#include "Util/ctpl_stl.h"

#include <vector>

namespace FusionCrowd
{
	class NeighborsSeeker
	{
	public:
		using SearchRequest = AgentSpatialInfo;

		struct SearchResult
		{
			size_t agentId;
			std::vector<NeighborInfo> neighbors;
			bool isOverlapped;
		};

	public:
		NeighborsSeeker();

		std::vector<SearchResult> FindNeighborsCpu(std::vector<SearchRequest> searchRequests);

	private:
		ctpl::thread_pool _pool;
	};
}
