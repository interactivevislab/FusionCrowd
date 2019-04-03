#pragma once
#include "../IOperComponent.h"
#include "../Math/vector.h"
#include "../NavComponents/Obstacle.h"
#include "../Agent.h"
#include "../Config.h"

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

			Math::Vector2 AgentForce(FusionCrowd::Agent* agent, const FusionCrowd::Agent * other) const;
			Math::Vector2 ObstacleForce(FusionCrowd::Agent* agent, const Obstacle * obst) const;
			Math::Vector2 DrivingForce(FusionCrowd::Agent* agent) const;
			void Update(FusionCrowd::Agent* agent, float timeStep);

		private:
			float _mass;
		};
	}
}
