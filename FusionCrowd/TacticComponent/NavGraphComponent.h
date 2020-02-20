#pragma once

#include "ITacticComponent.h"
#include "Simulator.h"
#include "Navigation/NavGraph/NavGraph.h"
#include "Navigation/NavSystem.h"
#include "Navigation/NavGraph/NavGraphPathPlanner.h"



namespace FusionCrowd
{
	class NavGraphComponent : public ITacticComponent
	{
	public:
		NavGraphComponent(
			std::shared_ptr<Simulator> simulator,
			std::shared_ptr<NavSystem> navSystem
		);

		void AddAgent(size_t id) override;
		bool DeleteAgent(size_t id) override;

		std::shared_ptr<NavGraph> GetNavGraph() const;

		void Update(float timeStep) override;
		DirectX::SimpleMath::Vector2 GetClosestAvailablePoint(DirectX::SimpleMath::Vector2 p) override;
		unsigned int getNodeId(size_t agentId) const;

		ComponentId GetId() override { return ComponentIds::NAVGRAPH_ID; }


	private:

		struct AgentStruct
		{
		public:
			unsigned int id;
			std::shared_ptr <NavGraphRoute> route;
			size_t goalNodeID;
			int nodesComplete;
		};

		void SetPrefVelocity(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct, float timeStep);
		void UpdateLocation(AgentSpatialInfo & agentInfo, AgentStruct& agentStruct, float deltaTime) const;
		void Replan(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct);
		size_t GetForwardAgent(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct);
		std::vector<AgentStruct> GetAllAgentsInRadius(AgentSpatialInfo & agentInfo, AgentStruct & agentStruct, float radius);

		std::shared_ptr<Simulator> _simulator;
		std::shared_ptr<NavGraph> _navGraph;
		std::shared_ptr<NavSystem> _navSystem;
		std::vector<AgentStruct> _agents;
		std::shared_ptr <NavGraphRoute> _route;
	};
}