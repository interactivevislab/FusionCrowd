#include "Agent.h"
#include "Navigation/SpatialQuery/SpatialQueryStructs.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	Agent::Agent()
	{
		_maxSpeed = 2.5f;
		_maxAccel = 2.f;
		_prefSpeed = 1.34f;
		_pos = Vector2(0.f, 0.f);
		_vel = Vector2(0.f, 0.f);
		_velPref = Agents::PrefVelocity(Vector2(1.f, 0.f), _prefSpeed, Vector2(0.f, 0.f));
		_velNew = Vector2(0.f, 0.f);
		_orient = Vector2(1.f, 0.f);
		_maxAngVel = MathUtil::TWOPI;	// 360 degrees/sec
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
		float speed = _vel.Length();
		const float speedThresh = _prefSpeed / 3.f;
		Vector2 newOrient(_orient);	// by default new is old
		Vector2 moveDir = _vel / speed;
		if (speed >= speedThresh) {
			newOrient = moveDir;
		}
		else {
			float frac = sqrtf(speed / speedThresh);
			Vector2 prefDir = _velPref.getPreferred();
			// prefDir *can* be zero if we've arrived at goal.  Only use it if it's non-zero.
			if (prefDir.LengthSquared() > 0.000001f) {
				newOrient = frac * moveDir + (1.f - frac) * prefDir;
				newOrient.Normalize();
			}
		}

		// Now limit angular velocity.
		const float MAX_ANGLE_CHANGE = timeStep * _maxAngVel;
		float maxCt = cos(MAX_ANGLE_CHANGE);
		float ct = newOrient.Dot(_orient);
		if (ct < maxCt) {
			// changing direction at a rate greater than _maxAngVel
			float maxSt = sin(MAX_ANGLE_CHANGE);
			if (MathUtil::det(_orient, newOrient) > 0.f) {
				// rotate _orient left
				_orient = Vector2(maxCt * _orient.x - maxSt * _orient.y, maxSt * _orient.x + maxCt * _orient.y);
			}
			else {
				// rotate _orient right
				_orient = Vector2(maxCt * _orient.x + maxSt * _orient.y, -maxSt * _orient.x + maxCt * _orient.y);
			}
		}
		else {
			_orient = newOrient;
		}
	}
}