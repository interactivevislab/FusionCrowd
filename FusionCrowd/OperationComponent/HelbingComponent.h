#pragma once
#include "IOperComponent.h"
#include "NavComponents/Obstacle.h"
#include "Agent.h"
#include "Config.h"
#include "Math/MathUtil.h"

namespace FusionCrowd
{
	namespace Helbing
	{
		class FUSION_CROWD_API HelbingComponent :
			public IOperComponent
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
