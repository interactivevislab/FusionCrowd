#pragma once

#include <string>

#include "ITacticComponent.h"
#include "Navigation/NavMesh/NavMesh.h"
#include "Navigation/NavMesh/NavMeshLocalizer.h"
#include "TacticComponent/Path/PrefVelocity.h"
#include "StrategyComponent/Goal/Goal.h"
#include "Agent.h"
#include "Config.h"
#include "Simulator.h"

namespace FusionCrowd
{
	class Simulator;
	class AgentSpatialInfo;

	class FUSION_CROWD_API NavMeshComponent : public ITacticComponent
	{
	public:
		NavMeshComponent(Simulator & simulator, const char* navMeshPath);

		void AddAgent(size_t id);
		bool RemoveAgent(size_t id);

		std::shared_ptr<NavMesh> GetNavMesh() { return _localizer->getNavMesh(); };
		std::shared_ptr<NavMeshLocalizer> GetLocalizer();

		void Update(float timeStep);
		void UpdateNavMesh(DirectX::SimpleMath::Vector2 point);
		unsigned int getNodeId(size_t agentId) const;
		unsigned int getNodeId(size_t agentId, const std::string& grpName, bool searchAll = false);

		~NavMeshComponent();

	private:
		struct AgentStruct
		{
		public:
			unsigned int id;
			NavMeshLocation location;
		};

		void setPrefVelocity(Agent & agent, AgentSpatialInfo & agentInfo, AgentStruct & agentStruct);
		unsigned int updateLocation(Agent & agent, AgentSpatialInfo & agentInfo, AgentStruct& agentStruct, bool force) const;

		/*
		PortalPath * getPath(size_t id);
		void setPath(size_t agentID, PortalPath * path);
		void clearPath(size_t agentID);
		void setNode(size_t agentID, unsigned int nodeID);
		*/

		Simulator & _simulator;

		float _headingDevCos;
		std::shared_ptr<NavMesh> _navMesh;
		std::shared_ptr<NavMeshLocalizer> _localizer;
		std::vector<AgentStruct> _agents;
	};
}

