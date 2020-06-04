#include "NeighborsSeeker.h"

#include "Math/consts.h"

#include <unordered_map>

using namespace DirectX::SimpleMath;

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
		std::future<NeighborsSeeker::SearchResult> result;
	};

	std::vector<NeighborsSeeker::SearchResult> NeighborsSeeker::FindNeighborsCpu(std::vector<NeighborsSeeker::SearchRequest> searchRequests)
	{
		std::vector<NeighborsSeeker::SearchResult> result;

		if(searchRequests.size() == 0)
			return result;

		float minX = Math::INFTY, maxX = -Math::INFTY;
		float minY = Math::INFTY, maxY = -Math::INFTY;

		for(const auto& r : searchRequests)
		{
			const Vector2 pos = r.GetPos();
			if(pos.x < minX)
				minX = pos.x;

			if(pos.x > maxX)
				maxX = pos.x;

			if(pos.y < minY)
				minY = pos.y;

			if(pos.y > maxY)
				maxY = pos.y;
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
			const int x = r.GetPos().x / cellSizeX;
			const int y = r.GetPos().y / cellSizeY;

			grid[{x, y}].push_back(r);
		}

		std::vector<Task> tasks;

		for(size_t i = 0; i < searchRequests.size(); i++)
		{
			auto lambda = [&grid, &reqs=searchRequests, reqIdx=i, cellSizeX, cellSizeY] (int threadId)
			{
				const SearchRequest& r = reqs.at(reqIdx);
				SearchResult result;
				result.agentId = r.id;

				const float R = r.neighbourSearchShape->BoundingRadius();
				const Vector2 pos = r.GetPos();

				int minCellX = (pos.x - R) / cellSizeX;
				int maxCellX = (pos.x + R) / cellSizeX + 1;

				int minCellY = (pos.y - R) / cellSizeY;
				int maxCellY = (pos.y + R) / cellSizeY + 1;

				for(int x = minCellX; x <= maxCellX; x++)
				{
					for(int y = minCellY; y <= maxCellY; y++)
					{
						const auto& cell = grid.find({x, y});
						if(cell == grid.end())
							continue;

						for(auto & n : cell->second)
						{
							if(r.CanCollide(n) && r.neighbourSearchShape->containsPoint(n.GetPos() - pos))
							{
								result.neighbors.push_back(NeighborInfo(n));
							}

							result.isOverlapped = result.isOverlapped || (Vector2::Distance(r.GetPos(), n.GetPos()) < (r.radius + n.radius));
						}
					}
				}

				return result;
			};

			tasks.push_back({searchRequests[i].id, _pool.push(lambda)});
		}

		for(auto & t : tasks)
		{
			result.push_back(t.result.get());
		}

		return result;
	}
}
