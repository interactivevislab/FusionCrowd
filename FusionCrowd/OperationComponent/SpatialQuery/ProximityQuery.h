#pragma once
#include "../../Config.h"
#include "../../Math/vector.h"
#include "../../NavComponents/Obstacle.h"

#include <vector>

namespace FusionCrowd
{
	//Forward Declaration
	class Agent;

	class FUSION_CROWD_API ProximityQuery
	{
	public:
		ProximityQuery() {}
		virtual ~ProximityQuery() {}

		virtual void StartQuery() = 0;
		virtual Math::Vector2 GetQueryPoint() = 0;
		virtual float GetMaxAgentRange() = 0;
		virtual float GetMaxObstacleRange() = 0;
		virtual void FilterAgent(const Agent * agent, float distSq) = 0;
		virtual void FilterObstacle(const Obstacle * obstacle, float distSq) = 0;
	};
}