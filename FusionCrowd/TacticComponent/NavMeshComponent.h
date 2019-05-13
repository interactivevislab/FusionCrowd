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

	class FUSION_CROWD_API NavMeshComponent : public ITacticComponent
	{
	public:
		NavMeshComponent(Simulator* simulator, const char* navMeshPath);

		void Update(float timeStep);

		unsigned int getNode(size_t agentId) const;
		unsigned int getNode(const FusionCrowd::Agent* agent, const std::string& grpName, bool searchAll = false);

		~NavMeshComponent();

	private:
		struct AgentStruct
		{
		public:
			unsigned int id;
			float radius;
			NavMeshLocation location;
			Agents::PrefVelocity prefVelocity;
		};

		void setPrefVelocity(FusionCrowd::Agent* agent, AgentStruct& agentStruct);
		unsigned int updateLocation(const FusionCrowd::Agent* agent, const AgentStruct& agentStruct, bool force) const;

		/*
		PortalPath * getPath(size_t id);
		void setPath(size_t agentID, PortalPath * path);
		void clearPath(size_t agentID);
		void setNode(size_t agentID, unsigned int nodeID);
		*/

		Simulator* _simulator;

		float _headingDevCos;
		NavMeshPtr _navMesh;
		NavMeshLocalizerPtr _localizer;
		std::vector<AgentStruct> _agents;
	};
}

