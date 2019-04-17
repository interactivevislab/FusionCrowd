#include "NavMeshComponent.h"
#include "Path/PortalPath.h"
#include "Path/PathPlanner.h"
#include "Path/Route.h"

using namespace DirectX::SimpleMath;

NavMeshComponent::NavMeshComponent()
{
}

void NavMeshComponent::SetPrefVelocity(const FusionCrowd::Agent* agent, const Goal * goal, Agents::PrefVelocity & pVel)
{
	PortalPath * path = _localizer->getPath(agent->_id);
	if (path == NULL) {
		Vector2 goalPoint = goal->getCentroid();
		unsigned int goalNode = _localizer->getNode(goalPoint);
		if (goalNode == NavMeshLocation::NO_NODE) {
			return;
		}
		unsigned int agtNode = _localizer->getNode(agent);
		PortalRoute * route = _localizer->getPlanner()->getRoute(agtNode, goalNode,
			agent->_radius * 2.f);

		path = new PortalPath(agent->_pos, goal, route, agent->_radius);
		// assign it to the localizer
		_localizer->setPath(agent->_id, path);
	}
	pVel.setSpeed(agent->_prefSpeed);
	path->setPreferredDirection(agent, _headingDevCos, pVel);
}


NavMeshComponent::~NavMeshComponent()
{
}
