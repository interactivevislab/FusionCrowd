#include "PathRandomization.h"
#include "PortalPath.h"
#include <random>

namespace FusionCrowd
{
	PathRandomization::PathRandomization()
	{
	}


	PathRandomization::~PathRandomization()
	{
	}

	using namespace DirectX::SimpleMath;

	std::mt19937 _rnd_seed;
	void PathRandomization::RandomizePath(FusionCrowd::PortalPath* path) {
		int portal_count = path->getPortalCount();
		for (int i = 0; i < portal_count; i++) {
			auto portal = path->getPortal(i);
			float width = Vector2::Distance(portal->getLeft(), portal->getRight());
			auto old_wp = path->getWayPoint(i);
			Vector2 random_wp = old_wp;
			float dl = Vector2::Distance(portal->getLeft(), old_wp);
			float dr = Vector2::Distance(portal->getRight(), old_wp);
			Vector2 delta = dl > dr ? portal->getLeft() - old_wp : portal->getRight() - old_wp;
			std::uniform_real_distribution<float> dist(0.0f, 0.6f);
			delta *= dist(_rnd_seed);
			path->setWaypoints(i, i + 1, old_wp + delta, path->_headings[i]);
		}
	}
}