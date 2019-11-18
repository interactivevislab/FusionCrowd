#pragma once
#include "Export/Config.h"
#include "Agent.h"
#include "Navigation/Obstacle.h"

#include <vector>

namespace FusionCrowd
{
	struct  NearAgent
	{
		/*!
		 * @brief		The squared distance from the query point to the agent.
		 */
		float distanceSquared;

		/*!
		 * @brief		A pointer to the agent at the indicated distance.
		 */
		const Agent * agent;

		/*!
		 * @brief		Constructor
		 *
		 * @param		sqdDist			The squared distance of the agent from a test point.
		 * @param		agt				The agent.
		 */
		NearAgent(float sqdDist, const Agent * agt) :distanceSquared(sqdDist), agent(agt) {};
	};

	struct  NearObstacle
	{
		/*!
		 * @brief		The squared distance from the query point to the obstacle.
		 */
		float distanceSquared;

		/*!
		 * @brief		A pointer to the obstacle at the indicated distance.
		 */
		const Obstacle * obstacle;

		/*!
		 * @brief		Constructor
		 *
		 * @param		sqdDist		The squared distance of the obstacle from a test point.
		 * @param		obs			The obstacle.
		 */
		NearObstacle(float sqdDist, const Obstacle * obs) :distanceSquared(sqdDist), obstacle(obs) {};
	};
}