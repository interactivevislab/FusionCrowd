#pragma once

#include "Agent.h"
#include "Config.h"
#include "Simulator.h"
#include "Navigation/NavSystem.h"
#include "OperationComponent/IOperationComponent.h"

#include <map>

namespace FusionCrowd
{
	namespace Helbing
	{
		struct FUSION_CROWD_API AgentParamentrs
		{
			float _mass;

			AgentParamentrs() :_mass(80.0f)
			{
			}
			AgentParamentrs(float mass) : _mass(mass)
			{
			}
		};

		class FUSION_CROWD_API HelbingComponent: public IOperationComponent
		{
		public:
			HelbingComponent(std::shared_ptr<NavSystem> navSystem);
			HelbingComponent(std::shared_ptr<NavSystem> navSystem, float AGENT_SCALE, float OBST_SCALE, float REACTION_TIME, float BODY_FORCE, float FRICTION, float FORCE_DISTANCE);
			~HelbingComponent();


			std::string GetName() { return "helbing"; };

			void AddAgent(size_t id);
			void AddAgent(size_t id, float mass);
			bool DeleteAgent(size_t id);

			void Update(float timeStep);

		private:
			void ComputeNewVelocity(AgentSpatialInfo & agent, float timeStep);
			DirectX::SimpleMath::Vector2 AgentForce(AgentSpatialInfo* agent, AgentSpatialInfo * other) const;
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