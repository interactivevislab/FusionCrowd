#pragma once

#include <memory>
#include <vector>
#include <functional>

#include "Math/Util.h"
#include "Export/Export.h"
#include "Export/IRecording.h"
#include "Export/INavSystemPublic.h"
#include "Util/spimpl.h"
#include "Navigation/NavMesh/NavMeshLocalizer.h"
#include "Navigation/NavGraph/NavGraph.h"
#include "TrafficLightsBunch.h"

namespace FusionCrowd
{
	class NavMeshComponent;
	class Obstacle;
	struct AgentSpatialInfo;

	class NavSystem : public INavSystemPublic
	{
	public:
		NavSystem();

		void SetNavMesh(std::shared_ptr<NavMeshLocalizer> localizer);
		void SetNavGraph(std::unique_ptr<NavGraph> navGraph);
		NavGraph* GetNavGraph();

		// Why do we need it?
		void Init();

		int CountNeighbors(size_t agentId) const; //TEST METHOD, MUST BE DELETED

		// Those looks very specific, are we sure they should be in public api?
		void SetAgentsSensitivityRadius(float radius);
		void SetGridCoeff(float coeff);

		// Very confusing methods pair
		void AddAgent(size_t agentId, DirectX::SimpleMath::Vector2 position);
		void AddAgent(AgentSpatialInfo spatialInfo);

		void AddTrafficLights(size_t nodeId);
		TrafficLightsBunch* GetTrafficLights(size_t nodeId);

		void RemoveAgent(unsigned int id);

		AgentSpatialInfo & GetSpatialInfo(size_t agentId);
		std::map<size_t, AgentSpatialInfo> GetAgentsSpatialInfos();

		const std::vector<AgentSpatialInfo> GetNeighbours(size_t agentId) const;
		std::vector<Obstacle> GetClosestObstacles(size_t agentId);

		void Update(float timeStep);

	public:
		// INavSystemPublic
		INavMeshPublic* GetPublicNavMesh() const;

		float CutPolygonFromMesh(FCArray<NavMeshVetrex> & polygon);
	private:
		class NavSystemImpl;

		spimpl::unique_impl_ptr<NavSystemImpl> pimpl;
	};
}
