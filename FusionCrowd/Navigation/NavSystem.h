#pragma once

#include <memory>
#include <vector>

#include "Export/Export.h"
#include "Export/IRecording.h"
#include "Export/INavSystemPublic.h"

#include "Math/Util.h"

#include "Navigation/NavMesh/NavMeshLocalizer.h"
#include "Navigation/NavGraph/NavGraph.h"
#include "Navigation/NeighborInfo.h"
#include "Navigation/AgentSpatialInfo.h"

#include "Util/spimpl.h"

namespace FusionCrowd
{
	class Obstacle;

	class NavSystem : public INavSystemPublic
	{
	public:
		NavSystem();

		void SetNavMesh(std::shared_ptr<NavMeshLocalizer> localizer);
		void SetNavGraph(std::unique_ptr<NavGraph> navGraph);
		NavGraph* GetNavGraph();

		// Why do we need it?
		void Init();

		void AddAgent(AgentSpatialInfo spatialInfo);

		void RemoveAgent(size_t id);

		AgentSpatialInfo & GetSpatialInfo(size_t agentId);

		std::vector<NeighborInfo> GetNeighbours(size_t agentId) const;
		std::vector<Obstacle> GetClosestObstacles(size_t agentId);

		void Update(float timeStep);

	public:
		// INavSystemPublic
		INavMeshPublic* GetPublicNavMesh() const;

		// Do we really need this method here?
		float CutPolygonFromMesh(FCArray<NavMeshVetrex> & polygon);
	private:
		class NavSystemImpl;

		spimpl::unique_impl_ptr<NavSystemImpl> pimpl;
	};
}
