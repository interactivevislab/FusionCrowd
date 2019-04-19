#pragma once
#include "OperationComponent/IOperationComponent.h"
#include "Navigation/Obstacle.h"
#include "Agent.h"
#include "Config.h"
#include "Math/Util.h"

#include <map>

namespace FusionCrowd
{
	namespace Helbing
	{
		struct FUSION_CROWD_API AgentParamentrs
		{
			float _mass;

			AgentParamentrs() :_mass(80.0)
			{
			}
			AgentParamentrs(float mass) :_mass(mass)
			{
			}
		};

		class FUSION_CROWD_API HelbingComponent :
			public IOperationComponent
		{
		public:
			HelbingComponent();
			HelbingComponent(float agentScale, float obstScale, float reactionTime, float bodyForce, float friction, float forceDistance);
			~HelbingComponent();

			void AddAgent(int idAgent, float mass);
			void DeleteAgent(int idAgent);

			void ComputeNewVelocity(FusionCrowd::Agent* agent);

			DirectX::SimpleMath::Vector2 AgentForce(FusionCrowd::Agent* agent, const FusionCrowd::Agent * other) const;
			DirectX::SimpleMath::Vector2 ObstacleForce(FusionCrowd::Agent* agent, const Obstacle * obst) const;
			DirectX::SimpleMath::Vector2 DrivingForce(FusionCrowd::Agent* agent) const;
			void Update(FusionCrowd::Agent* agent, float timeStep);

		private:
			std::map<int, AgentParamentrs> _agents;

			float _agentScale;
			float _obstScale;
			float _reactionTime;
			float _bodyForce;
			float _friction;
			float _forceDistance;
		};
	}
}
