/*#pragma once
#include "OperationComponent/IOperationComponent.h"
#include "Navigation/Obstacle.h"
#include "Agent.h"
#include "Config.h"
#include "Math/Util.h"

namespace FusionCrowd
{
	namespace Helbing
	{
		class FUSION_CROWD_API HelbingComponent :
			public IOperationComponent
		{
		public:
			HelbingComponent();
			~HelbingComponent();
			void ComputeNewVelocity(FusionCrowd::Agent* agent);

			DirectX::SimpleMath::Vector2 AgentForce(FusionCrowd::Agent* agent, const FusionCrowd::Agent * other) const;
			DirectX::SimpleMath::Vector2 ObstacleForce(FusionCrowd::Agent* agent, const Obstacle * obst) const;
			DirectX::SimpleMath::Vector2 DrivingForce(FusionCrowd::Agent* agent) const;
			void Update(FusionCrowd::Agent* agent, float timeStep);

		private:
			float _mass;
		};
	}
}
*/

#pragma once

#include "Agent.h"
#include "Config.h"
#include "Simulator.h"
#include "Navigation/NavSystem.h"
#include "OperationComponent/IOperationComponent.h"

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
			HelbingComponent(Simulator & simulator);
			HelbingComponent(Simulator & simulator, float AGENT_SCALE, float OBST_SCALE, float REACTION_TIME, float BODY_FORCE, float FRICTION, float FORCE_DISTANCE);
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


			Simulator & _simulator;
			NavSystem & _navSystem;
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