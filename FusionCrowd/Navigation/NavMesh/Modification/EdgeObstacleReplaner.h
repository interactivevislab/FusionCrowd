#pragma once

#include "../NavMesh.h"
#include "Navigation/SpatialQuery/NavMeshSpatialQuery.h"
#include <vector>

namespace FusionCrowd {
	class EdgeObstacleReplaner
	{
	public:
		EdgeObstacleReplaner(NavMesh& navmesh, std::shared_ptr<NavMeshLocalizer> localizer);
		~EdgeObstacleReplaner();
		void Replan();
	private:
		const float DELTA = 1e-3;
		NavMesh& _navmesh;
		std::shared_ptr<NavMeshLocalizer> _localizer;
		std::vector<NavMeshEdge*> _edges;
		std::vector<NavMeshObstacle*> _obstacles;

		void ProcessNode(NavMeshNode& node);
		bool ValidateEdge(NavMeshEdge* edge);
		void FillNavmesh();
	};
}

