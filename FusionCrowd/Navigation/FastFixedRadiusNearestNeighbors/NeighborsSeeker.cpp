#include "NeighborsSeeker.h"

namespace FusionCrowd
{
	NeighborsSeeker::NeighborsSeeker()
	{
	}

	NeighborsSeeker::SearchResult NeighborsSeeker::FindNeighborsCpuSquare(std::vector<NeighborsSeeker::SearchRequest> searchRequests)
	{
		NeighborsSeeker::SearchResult result;

		if(searchRequests.size() == 0)
			return result;

		for(size_t i = 0; i < searchRequests.size() - 1; i++)
		{
			for(size_t j = i + 1; j < searchRequests.size(); j++)
			{
				if(i == j) continue;

				auto & req1 = searchRequests[i];
				auto & req2 = searchRequests[j];

				if(req1.neighbourSearchShape->containsPoint(req2.pos - req1.pos))
				{
					result[req1.id].push_back(req2.id);
				}

				if(req2.neighbourSearchShape->containsPoint(req1.pos - req2.pos))
				{
					result[req2.id].push_back(req1.id);
				}
			}
		}

		return result;
	}
}
