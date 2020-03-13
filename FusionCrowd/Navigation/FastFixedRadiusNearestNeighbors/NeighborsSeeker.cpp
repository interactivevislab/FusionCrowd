#include "NeighborsSeeker.h"
#include "GpuCalculator.h"
#include <iostream>
#include <string>
#include "Math/Util.h"

namespace FusionCrowd
{
	using namespace DirectX::SimpleMath;

	NeighborsSeeker::NeighborsSeeker()
	{
	}
	NeighborsSeeker::~NeighborsSeeker()
	{
	}

	std::vector<NeighborsSeeker::PointNeighbors> NeighborsSeeker::FindNeighborsCpuSquare(std::vector<AgentSpatialInfo> positions, std::vector<Math::Geometry2D*> search_areas) {
		using namespace DirectX::SimpleMath;
		std::vector<PointNeighbors> res;
		for (int i = 0; i < positions.size(); i++) {
			PointNeighbors pn;
			pn.pointID = i;
			res.push_back(pn);
		}


		for (int i = 0; i < positions.size(); i++) {
			Math::Geometry2D& geometry = *search_areas[i]; // i = id
			PointNeighbors& neighbors = res[i];
			for (int j = 0; j < positions.size(); j++) {
				if (i == j) continue;
				if (geometry.containsPoint(positions[j].pos, positions[i].pos, atan2(positions[i].orient.y, positions[i].orient.x))) {
					neighbors.neighborsID.push_back(j);
				}
			}
		}
		return res;
	}
#pragma endregion

}