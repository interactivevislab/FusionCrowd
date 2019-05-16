#include "Agent.h"
#include "Navigation/SpatialQuery/SpatialQueryStructs.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	Agent::Agent(Goal& goal) : _currentGoal(goal)
	{
		id = 0;
		maxSpeed = 2.5f;
		maxAccel = 2.f;
		prefSpeed = 1.34f;
		pos = Vector2(0.f, 0.f);
		vel = Vector2(0.f, 0.f);

		prefVelocity = Agents::PrefVelocity(Vector2(1.f, 0.f), prefSpeed, Vector2(0.f, 0.f));

		velNew = Vector2(0.f, 0.f);
		orient = Vector2(1.f, 0.f);
		maxAngVel = MathUtil::TWOPI; // 360 degrees/sec

		radius = 0.19f;
	}

	Agent::~Agent()
	{
	}

	/*
	void Agent::StartQuery() {
		_nearAgents.clear();
		_nearObstacles.clear();
	};

	float Agent::GetMaxAgentRange()
	{
		if (_nearAgents.size() == _maxNeighbors) {
			return _nearAgents.back().distanceSquared;
		}

		return _neighborDist * _neighborDist;
	}

	void Agent::FilterAgent(const Agent *agent, float distance)
	{
		InsertAgentNeighbor(agent, distance);
	};

	void Agent::FilterObstacle(const Obstacle * obstacle, float distance) {
		InsertObstacleNeighbor(obstacle, distance);
	};

	void Agent::InsertAgentNeighbor(const Agent* agent, float distSq) {
		if (this != agent) {
			if (_nearAgents.size() != _maxNeighbors || distSq <= GetMaxAgentRange()) {
				if (_nearAgents.size() != _maxNeighbors) {
					_nearAgents.push_back(NearAgent(distSq, agent));
				}
				size_t i = _nearAgents.size() - 1;
				while (i != 0 && distSq < _nearAgents[i - 1].distanceSquared) {
					_nearAgents[i] = _nearAgents[i - 1];
					--i;
				}
				_nearAgents[i] = NearAgent(distSq, agent);

			}
		}
	}

	void Agent::InsertObstacleNeighbor(const Obstacle* obstacle, float distSq) {
		// the assumption is that two obstacle neighbors MUST have the same classID
		if (obstacle->_class & _obstacleSet) {

			if (distSq < _neighborDist * _neighborDist) {

				_nearObstacles.push_back(NearObstacle(distSq, obstacle));

				size_t i = _nearObstacles.size() - 1;
				while (i != 0 && distSq < _nearObstacles[i - 1].distanceSquared) {
					_nearObstacles[i] = _nearObstacles[i - 1];
					--i;
				}
				_nearObstacles[i] = NearObstacle(distSq, obstacle);
			}
		}
	}
	*/


	void Agent::UpdateOrient(float timeStep)
	{
		float speed = vel.Length();
		const float speedThresh = prefSpeed / 3.f;
		Vector2 newOrient(orient); // by default new is old
		Vector2 moveDir = vel / speed;
		if (speed >= speedThresh)
		{
			newOrient = moveDir;
		}
		else
		{
			float frac = sqrtf(speed / speedThresh);
			Vector2 prefDir = prefVelocity.getPreferred();
			// prefDir *can* be zero if we've arrived at goal.  Only use it if it's non-zero.
			if (prefDir.LengthSquared() > 0.000001f)
			{
				newOrient = frac * moveDir + (1.f - frac) * prefDir;
				newOrient.Normalize();
			}
		}

		// Now limit angular velocity.
		const float MAX_ANGLE_CHANGE = timeStep * maxAngVel;
		float maxCt = cos(MAX_ANGLE_CHANGE);
		float ct = newOrient.Dot(orient);
		if (ct < maxCt)
		{
			// changing direction at a rate greater than _maxAngVel
			float maxSt = sin(MAX_ANGLE_CHANGE);
			if (MathUtil::det(orient, newOrient) > 0.f)
			{
				// rotate _orient left
				orient = Vector2(maxCt * orient.x - maxSt * orient.y, maxSt * orient.x + maxCt * orient.y);
			}
			else
			{
				// rotate _orient right
				orient = Vector2(maxCt * orient.x + maxSt * orient.y, -maxSt * orient.x + maxCt * orient.y);
			}
		}
		else
		{
			orient = newOrient;
		}
	}

	Goal& Agent::getCurrentGoal() const
	{
		return _currentGoal;
	}
}
