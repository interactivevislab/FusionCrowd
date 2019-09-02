#pragma once

#include <memory>
#include <vector>
#include <functional>

#include "Config.h"
#include "Math/Util.h"
#include "Util/PublicSpatialInfo.h"
#include "Util/IRecording.h"
#include "Navigation/NavMesh/NavMeshLocalizer.h"

namespace FusionCrowd
{
	class NavMeshComponent;
	class Obstacle;
	struct AgentSpatialInfo;

	class FUSION_CROWD_API NavSystem
	{
	public:
		NavSystem(std::shared_ptr<NavMeshLocalizer> localizer);

		virtual ~NavSystem();

		NavSystem(const NavSystem &) = delete;
		NavSystem& operator=(const NavSystem&) = delete;

		NavSystem(NavSystem&&);
		NavSystem& operator=(NavSystem&&);

		// Why do we need it?
		void Init();

		PublicSpatialInfo GetPublicSpatialInfo(size_t agentId);
		int CountNeighbors(size_t agentId) const; //TEST METHOD, MUST BE DELETED

		// Those looks very specific, are we sure they should be in public api?
		void SetAgentsSensitivityRadius(float radius);
		void SetGridCoeff(float coeff);

		IRecording* GetRecording();

		// Very confusing methods pair
		void AddAgent(size_t agentId, DirectX::SimpleMath::Vector2 position);
		void AddAgent(AgentSpatialInfo spatialInfo);

		AgentSpatialInfo & GetSpatialInfo(size_t agentId);

		const std::vector<AgentSpatialInfo> & GetNeighbours(size_t agentId) const;
		std::vector<Obstacle> GetClosestObstacles(size_t agentId) const;

		void Update(float timeStep);
	private:
		class NavSystemImpl;

		std::unique_ptr<NavSystemImpl> pimpl;
	};
}
