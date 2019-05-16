#pragma once

#include "Config.h"

#include "TacticComponent/Path/PrefVelocity.h"

#include <vector>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	class Goal;

	class FUSION_CROWD_API Agent
	{
	public:
		Agent(Goal & goal);
		~Agent();

		size_t id;
		float radius;
		float maxSpeed;
		float maxAccel;
		float prefSpeed;
		float maxAngVel;
		Vector2 pos;
		Vector2 vel;
		Vector2 velNew;
		Vector2 orient;
		Agents::PrefVelocity prefVelocity;

		Goal & getCurrentGoal() const;

		void UpdateOrient(float timeStep);
		void PostUpdate() { };

		/*
		void StartQuery();
		Vector2 GetQueryPoint() { return _pos; };
		float GetMaxAgentRange();
		float GetMaxObstacleRange() { return _neighborDist * _neighborDist; };
		void FilterAgent(const Agent *agent, float distance);
		void FilterObstacle(const Obstacle * obstacle, float distance);

		void InsertAgentNeighbor(const Agent* agent, float distSq);
		void InsertObstacleNeighbor(const Obstacle* obstacle, float distSq);

		int _maxNeighbors;
		float _radius;
		float _priority;
		float _neighborDist;
		size_t _obstacleSet;

		std::vector<NearAgent> _nearAgents;
		std::vector<NearObstacle> _nearObstacles;
		*/

	private:
		std::reference_wrapper<Goal> _currentGoal;
	};
}