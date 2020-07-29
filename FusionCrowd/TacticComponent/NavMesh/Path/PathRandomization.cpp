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
			std::uniform_real_distribution<float> dist(0.0f, 0.3f);
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

	void PathRandomization::RandomizePath(FusionCrowd::PortalPath* path, float customEdgePosition, float customDistribution, float agentRadius, const std::shared_ptr<NavMesh> navMesh)
	{
		int portal_count = path->getPortalCount();
		RandomizePath(path, 0, portal_count, customEdgePosition, customDistribution, agentRadius, navMesh);
	}

	void PathRandomization::RandomizePath(FusionCrowd::PortalPath* path, size_t start, size_t end, float customEdgePosition, float customDistribution, float agentRadius, const std::shared_ptr<NavMesh> navMesh)
	{
		int portal_count = path->getPortalCount();
		for (int i = start; i < end; i++)
		{
			auto portal = path->getPortal(i);

			unsigned int currNodeID = path->getNode();
			const NavMeshNode* currNode = &(navMesh->GetNodeByPos(currNodeID));
			bool isLeft = false;
			float leftOffset = 0.0f;
			float rightOffset = 0.0f;
			for (size_t i = 0; i < currNode->_obstCount; i++)
			{
				Vector2 pDir;
				if (currNode->getObstacle(i)->getP0() == portal->getLeft())
				{
					pDir = (portal->getRight() - portal->getLeft());
					isLeft = true;
				}
				else
				{
					pDir = (portal->getLeft() - portal->getRight());
					isLeft = false;
				}

				auto oDir = currNode->getObstacle(i)->_unitDir;
				pDir.Normalize();
				auto dot = oDir.Dot(pDir);
				auto oLen = oDir.Length();
				auto pLen = pDir.Length();
				auto magn = oLen * pLen;
				auto cos = dot / magn;

				if (isLeft)
					leftOffset = agentRadius / cos;
				else
					rightOffset = agentRadius / cos;
			}

			const NavMeshNode* nextNode = 0x0;
			if (i + 1 < portal_count)
			{
				// there is another way portal to test
				const WayPortal* nextPortal = path->getPortal(i + 1);
				size_t nextID = nextPortal->getNodeId();
				nextNode = &(navMesh->GetNodeByPos((unsigned int)nextID));
			}
			else if (i < portal_count)
			{
				// the next node is the goal polygon
				nextNode = &(navMesh->GetNodeByPos((unsigned int)path->getEndNode()));
			}
			for (size_t i = 0; i < nextNode->_obstCount; i++)
			{
				Vector2 pDir;
				if (nextNode->getObstacle(i)->getP0() == portal->getLeft())
				{
					pDir = (portal->getRight() - portal->getLeft());
					isLeft = true;
				}
				else
				{
					pDir = (portal->getLeft() - portal->getRight());
					isLeft = false;
				}

				auto oDir = nextNode->getObstacle(i)->_unitDir;
				pDir.Normalize();
				auto dot = oDir.Dot(pDir);
				auto oLen = oDir.Length();
				auto pLen = pDir.Length();
				auto magn = oLen * pLen;
				auto cos = dot / magn;

				if (isLeft)
					leftOffset = agentRadius / cos;
				else
					rightOffset = agentRadius / cos;
			}





			float width = Vector2::Distance(portal->getLeft(), portal->getRight());
			auto old_wp = portal->getLeft(width * customEdgePosition);
			Vector2 delta = (portal->getLeft() - portal->getRight()) / 2;
			std::uniform_real_distribution<float> dist(-customDistribution, customDistribution);
			delta *= dist(_rnd_seed);
			auto newWaypoint = old_wp + delta;
			float newToLeft = Vector2::Distance(portal->getLeft(), newWaypoint);
			float newToRight = Vector2::Distance(newWaypoint, portal->getRight());
			newWaypoint =
				newToLeft > (width - rightOffset*2) ? portal->getRight(rightOffset*2) :
				newToRight > (width - leftOffset*2) ? portal->getLeft(leftOffset*2) :
				newWaypoint;

			path->setWaypoints(i, i + 1, newWaypoint, path->_headings[i]);
		}
	}
}