#include "Agent.h"
#include "Navigation/SpatialQuery/SpatialQueryStructs.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	Agent::Agent(size_t agentId) : id(agentId)
	{
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
}
