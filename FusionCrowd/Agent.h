#pragma once

#include "OperationComponent/SpatialQuery/SpatialQuery.h"
#include "OperationComponent/SpatialQuery/SpatialQueryStructs.h"
#include "Math/vector.h"
#include "Path/PrefVelocity.h"
#include "Config.h"

#include <vector>

namespace FusionCrowd
{

	class FUSION_CROWD_API Agent : public ProximityQuery
	{
	public:
		Agent();
		~Agent();

		void SetPreferredVelocity(Agents::PrefVelocity &velocity);

		void StartQuery();
		Math::Vector2 GetQueryPoint() { return _pos; };
		float GetMaxAgentRange();
		float GetMaxObstacleRange() { return _neighborDist * _neighborDist; };
		void FilterAgent(const Agent *agent, float distance);
		void FilterObstacle(const Obstacle * obstacle, float distance);

		void InsertAgentNeighbor(const Agent* agent, float distSq);
		void InsertObstacleNeighbor(const Obstacle* obstacle, float distSq);
		void UpdateOrient(float timeStep);
		void PostUpdate() {}

		int _id;
		int _maxNeighbors;
		float _maxSpeed;
		float _maxAccel;
		float _prefSpeed;
		Math::Vector2 _pos;
		Math::Vector2 _vel;
		Agents::PrefVelocity _velPref;
		Math::Vector2 _velNew;
		Math::Vector2 _orient;
		float _radius;
		float _priority;
		float _neighborDist;
		float _maxAngVel;
		size_t _obstacleSet;

		std::vector<NearAgent> _nearAgents;
		std::vector<NearObstacle> _nearObstacles;
	};
}