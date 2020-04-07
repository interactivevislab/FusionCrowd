#pragma once

#include "Agent.h"
#include "Simulator.h"
#include "Navigation/NavSystem.h"
#include "OperationComponent/IOperationComponent.h"
#include "Export/ComponentId.h"

#include "Navigation/NeighborInfo.h"
#include "Navigation/AgentSpatialInfo.h"

#include <map>

namespace FusionCrowd
{
	namespace Helbing
	{
		struct AgentParamentrs
		{
			float _mass;

			AgentParamentrs() :_mass(80.0f)
			{
			}
			AgentParamentrs(float mass) : _mass(mass)
			{
			}
		};

		class HelbingComponent: public IOperationComponent
		{
		public:
			HelbingComponent(std::shared_ptr<NavSystem> navSystem);
			HelbingComponent(std::shared_ptr<NavSystem> navSystem, float AGENT_SCALE, float OBST_SCALE, float REACTION_TIME, float BODY_FORCE, float FRICTION, float FORCE_DISTANCE);
			~HelbingComponent();

			ComponentId GetId() override { return ComponentIds::HELBING_ID; };

			void AddAgent(size_t id) override;
			void AddAgent(size_t id, float mass);
			bool DeleteAgent(size_t id) override;

			void Update(float timeStep) override;

		private:
			void ComputeNewVelocity(AgentSpatialInfo & agent, float timeStep);
			DirectX::SimpleMath::Vector2 AgentForce(AgentSpatialInfo* agent, NeighborInfo * other) const;
			DirectX::SimpleMath::Vector2 ObstacleForce(AgentSpatialInfo* agent, Obstacle * obst) const;
			DirectX::SimpleMath::Vector2 DrivingForce(AgentSpatialInfo* agent);


			std::shared_ptr<NavSystem> _navSystem;
			std::map<int, AgentParamentrs> _agents;

			float _agentScale;
			float _obstScale;
			float _reactionTime;
			float _bodyForse;
			float _friction;
			float _forceDistance;
		};
	}
}