#include "NeighborsSeeker.h"

#include "Math/Util.h"

#include <unordered_map>

namespace FusionCrowd
{
	NeighborsSeeker::NeighborsSeeker() : _pool(std::thread::hardware_concurrency() - 1)
	{
	}

	struct Cell
	{
		int x;
		int y;
	};

	struct CellHash
    {
        size_t operator()(FusionCrowd::Cell const& c) const noexcept
        {
            return c.x ^ (c.y << 1);
        }
    };

	bool operator==(const Cell & l, const Cell& r)
	{
		return l.x == r.x && l.y == r.y;
	}

	struct Task
	{
		size_t id;
		std::future<std::vector<NeighborInfo>> result;
	};

	NeighborsSeeker::SearchResult NeighborsSeeker::FindNeighborsCpu(std::vector<NeighborsSeeker::SearchRequest> searchRequests)
	{
		NeighborsSeeker::SearchResult result;

		if(searchRequests.size() == 0)
			return result;

		float minX = MathUtil::INFTY, maxX = -MathUtil::INFTY;
		float minY = MathUtil::INFTY, maxY = -MathUtil::INFTY;

		for(const auto& r : searchRequests)
		{
			if(r.pos.x < minX)
				minX = r.pos.x;

			if(r.pos.x > maxX)
				maxX = r.pos.x;

			if(r.pos.y < minY)
				minY = r.pos.y;

			if(r.pos.y > maxY)
				maxY = r.pos.y;
		}

		// Total cells             = n^(1/4) * n^(1/4) = n*(1/2)
		// Avg agents in each cell = n / totalCells    = n^(1/2)
		// Search complexity using cells = O(n^(1/2) * n^(1/2) * searchR^2) = O(n)
		// Overall neighbours search complexity = totalCells * searchCmplxInCell = O(n^(1/2) * n) = O(n^3/2)
		const size_t cellsAlongDimension = sqrt(sqrt(searchRequests.size())) + 1;

		const int cellSizeX = (maxX - minX) / cellsAlongDimension;
		const int cellSizeY = (maxY - minY) / cellsAlongDimension;

		std::unordered_map<Cell, std::vector<SearchRequest>, CellHash> grid;

		for(const auto& r : searchRequests)
		{
			const int x = r.pos.x / cellSizeX;
			const int y = r.pos.y / cellSizeY;

			grid[{x, y}].push_back(r);
		}

		std::vector<Task> tasks;

		for(const auto& r : searchRequests)
		{
			auto lambda = [&grid, r=r, cellSizeX, cellSizeY] (int threadId)
			{
				std::vector<NeighborInfo> result;
				const float R = r.neighbourSearchShape->BoundingRadius();
				int minCellX = (r.pos.x - R) / cellSizeX;
				int maxCellX = (r.pos.x + R) / cellSizeX + 1;

				int minCellY = (r.pos.y - R) / cellSizeY;
				int maxCellY = (r.pos.y + R) / cellSizeY + 1;

				for(int x = minCellX; x <= maxCellX; x++)
				{
					for(int y = minCellY; y <= maxCellY; y++)
					{
						const auto& cell = grid.find({x, y});
						if(cell == grid.end())
							continue;

						for(auto & n : cell->second)
						{
							if(r.neighbourSearchShape->containsPoint(n.pos - r.pos))
							{
								result.push_back(NeighborInfo(n));
							}
						}
					}
				}

				return result;
			};

			tasks.push_back({r.id, _pool.push(lambda)});
		}

		for(auto & t : tasks)
		{
			result[t.id] = t.result.get();
		}

		return result;
	}
}
