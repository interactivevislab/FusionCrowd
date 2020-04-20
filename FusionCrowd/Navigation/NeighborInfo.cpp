#include "NeighborInfo.h"

namespace FusionCrowd
{
	NeighborInfo::NeighborInfo(const AgentSpatialInfo& agent) :
			id(agent.id),
			pos(agent.GetPos()),
			orient(agent.GetOrient()),
			vel(agent.GetVel()),
			radius(agent.radius),
			inertiaEnabled(agent.inertiaEnabled),
			collisionsLevel(agent.collisionsLevel),
			prefVel(agent.prefVelocity.getPreferredVel())
	{}
}
