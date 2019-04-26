#pragma once

#include "Navigation/SpatialQuery/SpatialQuery.h"
#include "Navigation/SpatialQuery/SpatialQueryStructs.h"
#include "TacticComponent/Path/PrefVelocity.h"
#include "Config.h"

#include <vector>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{

	class FUSION_CROWD_API Agent : public ProximityQuery
	{
	public:
		Agent();
		~Agent();

		void SetPreferredVelocity(Agents::PrefVelocity &velocity);

		void StartQuery();
		Vector2 GetQueryPoint() { return _pos; };
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
		Vector2 _pos;
		Vector2 _vel;
		Agents::PrefVelocity _velPref;
		Vector2 _velNew;
		Vector2 _orient;
		float _radius;
		float _priority;
		float _neighborDist;
		float _maxAngVel;
		size_t _obstacleSet;
		int _operationComponent;

		std::vector<NearAgent> _nearAgents;
		std::vector<NearObstacle> _nearObstacles;
	};
}