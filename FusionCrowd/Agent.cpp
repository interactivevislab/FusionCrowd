#include "Agent.h"
#include "OperationComponent/SpatialQuery/SpatialQueryStructs.h"

namespace FusionCrowd
{
	Agent::Agent()
	{
		_maxSpeed = 2.5f;
		_maxAccel = 2.f;
		_prefSpeed = 1.34f;
		_pos = Math::Vector2(0.f, 0.f);
		_vel = Math::Vector2(0.f, 0.f);
		_velPref = Agents::PrefVelocity(Math::Vector2(1.f, 0.f), _prefSpeed, Math::Vector2(0.f, 0.f));
		_velNew = Math::Vector2(0.f, 0.f);
		_orient = Math::Vector2(1.f, 0.f);
		_maxAngVel = TWOPI;	// 360 degrees/sec
		_maxNeighbors = 10;
		_neighborDist = 5.f;
		_nearAgents.clear();
		_nearObstacles.clear();
		_priority = 0.f;
		_id = 0;
		_radius = 0.19f;
	}

	Agent::~Agent()
	{
	}

	void Agent::StartQuery() {
		_nearAgents.clear();
		_nearObstacles.clear();
	};

	void Agent::SetPreferredVelocity(Agents::PrefVelocity &velocity)
	{
		//set my velocity to be the given one
		_velPref = velocity;
	}

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

	void Agent::UpdateOrient(float timeStep)
	{
		float speed = abs(_vel);
		const float speedThresh = _prefSpeed / 3.f;
		Math::Vector2 newOrient(_orient);	// by default new is old
		Math::Vector2 moveDir = _vel / speed;
		if (speed >= speedThresh) {
			newOrient = moveDir;
		}
		else {
			float frac = sqrtf(speed / speedThresh);
			Math::Vector2 prefDir = _velPref.getPreferred();
			// prefDir *can* be zero if we've arrived at goal.  Only use it if it's non-zero.
			if (absSq(prefDir) > 0.000001f) {
				newOrient = frac * moveDir + (1.f - frac) * prefDir;
				newOrient.normalize();
			}
		}

		// Now limit angular velocity.
		const float MAX_ANGLE_CHANGE = timeStep * _maxAngVel;
		float maxCt = cos(MAX_ANGLE_CHANGE);
		float ct = newOrient * _orient;
		if (ct < maxCt) {
			// changing direction at a rate greater than _maxAngVel
			float maxSt = sin(MAX_ANGLE_CHANGE);
			if (det(_orient, newOrient) > 0.f) {
				// rotate _orient left
				_orient.set(maxCt * _orient._x - maxSt * _orient._y,
					maxSt * _orient._x + maxCt * _orient._y);
			}
			else {
				// rotate _orient right
				_orient.set(maxCt * _orient._x + maxSt * _orient._y,
					-maxSt * _orient._x + maxCt * _orient._y);
			}
		}
		else {
			_orient.set(newOrient);
		}
	}
}