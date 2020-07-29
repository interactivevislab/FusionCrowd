#include "PathRandomization.h"
#include "PortalPath.h"
#include <random>

#include "Navigation/AgentSpatialInfo.h"

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
	void PathRandomization::RandomizePath(FusionCrowd::PortalPath* path, size_t start, size_t end) {
		int portal_count = path->getPortalCount();
		for (int i = start; i < end; i++) {
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

	void PathRandomization::RandomizePath(FusionCrowd::PortalPath* path)
	{
		size_t start = 0;
		int portal_count = path->getPortalCount();
		RandomizePath(path, start, portal_count);
	}

	void PathRandomization::RandomizePath(FusionCrowd::PortalPath* path, float customEdgePosition, float customDistribution, float agentRadius)
	{
		int portal_count = path->getPortalCount();
		RandomizePath(path, 0, portal_count, customEdgePosition, customDistribution, agentRadius);
	}

	void PathRandomization::RandomizePath(FusionCrowd::PortalPath* path, size_t start, size_t end, float customEdgePosition, float customDistribution, float agentRadius)
	{
		int portal_count = path->getPortalCount();
		for (int i = start; i < end; i++)
		{
			auto portal = path->getPortal(i);
			float width = Vector2::Distance(portal->getLeft(), portal->getRight());
			auto old_wp = portal->getLeft(width * customEdgePosition);
			Vector2 delta = (portal->getLeft() - portal->getRight()) / 2;
			std::uniform_real_distribution<float> dist(-customDistribution, customDistribution);
			delta *= dist(_rnd_seed);
			auto newWaypoint = old_wp + delta;
			float newToLeft = Vector2::Distance(portal->getLeft(), newWaypoint);
			float newToRight = Vector2::Distance(newWaypoint, portal->getRight());
			newWaypoint =
				newToLeft > width ? portal->getRight(agentRadius) : 
				newToRight > width ? portal->getLeft(agentRadius) : 
				newWaypoint;

			path->setWaypoints(i, i + 1, newWaypoint, path->_headings[i]);
		}
	}
}