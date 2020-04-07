#include "NeighborInfo.h"

namespace FusionCrowd
{
	NeighborInfo::NeighborInfo(const AgentSpatialInfo& agent) :
			id(agent.id),
			pos(agent.pos),
			orient(agent.orient),
			vel(agent.vel),
			radius(agent.radius),
			inertiaEnabled(agent.inertiaEnabled),
			collisionsLevel(agent.collisionsLevel),
			prefVel(agent.prefVelocity.getPreferredVel())
	{}
}
