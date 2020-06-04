#pragma once

#include <string>

#include "Export/ComponentId.h"
#include "TacticComponent/ITacticComponent.h"

#include "Navigation/NavMesh/NavMesh.h"
#include "Navigation/NavMesh/NavMeshLocalizer.h"
#include "Navigation/SpatialQuery/NavMeshSpatialQuery.h"
#include "Simulator.h"

namespace FusionCrowd
{
	class Simulator;
	class AgentSpatialInfo;

	class NavMeshComponent : public ITacticComponent
	{
	public:
		NavMeshComponent(
			std::shared_ptr<Simulator> simulator,
			std::shared_ptr<NavMeshLocalizer> localizer,
			std::shared_ptr<NavMeshSpatialQuery> spatial_query
		);

		void AddAgent(size_t id) override;
		bool DeleteAgent(size_t id) override;

		std::shared_ptr<NavMesh> GetNavMesh() const;
		std::shared_ptr<NavMeshLocalizer> GetLocalizer() const;

		void Update(float timeStep) override;
		DirectX::SimpleMath::Vector2 GetClosestAvailablePoint(DirectX::SimpleMath::Vector2 p) override;
		size_t GetClosestAvailableNode(DirectX::SimpleMath::Vector2 p);
		size_t getNodeId(size_t agentId) const;

		ComponentId GetId() override { return ComponentIds::NAVMESH_ID; }

	private:
		struct AgentStruct
		{
		public:
			unsigned int id;
			NavMeshLocation location;
		};

		NavMeshLocation Replan(DirectX::SimpleMath::Vector2 from, const Goal & target, float agentRadius);
		bool IsReplanNeeded(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct);

		void SetPrefVelocity(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct, float timeStep);
		unsigned int UpdateLocation(AgentSpatialInfo & agentInfo, AgentStruct& agentStruct, bool force) const;

	private:
		std::shared_ptr<Simulator> _simulator;

		float _headingDevCos;
		std::shared_ptr<NavMesh> _navMesh;
		std::shared_ptr<NavMeshLocalizer> _localizer;
		std::shared_ptr<NavMeshSpatialQuery> _spatial_query;
		std::vector<AgentStruct> _agents;
	};
}

