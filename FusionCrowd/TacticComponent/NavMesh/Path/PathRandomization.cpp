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

			unsigned int currNodeID = portal->getNodeId();
			const NavMeshNode* currNode = &(navMesh->GetNodeByPos(currNodeID));
			bool isLeft = false;
			float leftOffset = agentRadius;
			float rightOffset = agentRadius;

			for (size_t j = 0; j < currNode->_obstCount; j++)
			{
				Vector2 pDir;
				Vector2 oDir;
				if (currNode->getObstacle(j)->getP0() == portal->getLeft())
				{
					pDir = (portal->getRight() - portal->getLeft());
					oDir = currNode->getObstacle(j)->_unitDir;
					isLeft = true;
				}
				else if (currNode->getObstacle(j)->getP0() == portal->getRight())
				{
					oDir = currNode->getObstacle(j)->_unitDir;
					pDir = (portal->getLeft() - portal->getRight());
					isLeft = false;
				}
				else if (currNode->getObstacle(j)->getP1() == portal->getLeft())
				{
					oDir = -currNode->getObstacle(j)->_unitDir;
					pDir = (portal->getRight() - portal->getLeft());
					isLeft = true;
				}
				else if (currNode->getObstacle(j)->getP1() == portal->getRight())
				{
					oDir = -currNode->getObstacle(j)->_unitDir;
					pDir = (portal->getLeft() - portal->getRight());
					isLeft = false;
				}
				else
				{
					continue;
				}
				pDir.Normalize();
				auto dot = oDir.Dot(pDir);
				auto oLen = oDir.Length();
				auto pLen = pDir.Length();
				auto magn = oLen * pLen;
				auto cos = dot / magn;
				auto sin = sqrt(1 - cos * cos);

				if (isLeft)
					leftOffset = std::max(leftOffset, agentRadius / sin);
				else
					rightOffset = std::max(rightOffset, agentRadius / sin);
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
			for (size_t j = 0; j < nextNode->_obstCount; j++)
			{
				Vector2 pDir;
				Vector2 oDir;
				if (nextNode->getObstacle(j)->getP0() == portal->getLeft())
				{
					pDir = (portal->getRight() - portal->getLeft());
					oDir = nextNode->getObstacle(j)->_unitDir;
					isLeft = true;
				}
				else if (nextNode->getObstacle(j)->getP0() == portal->getRight())
				{
					oDir = nextNode->getObstacle(j)->_unitDir;
					pDir = (portal->getLeft() - portal->getRight());
					isLeft = false;
				}
				else if (nextNode->getObstacle(j)->getP1() == portal->getLeft())
				{
					oDir = -nextNode->getObstacle(j)->_unitDir;
					pDir = (portal->getRight() - portal->getLeft());
					isLeft = true;
				}
				else if (nextNode->getObstacle(j)->getP1() == portal->getRight())
				{
					oDir = -nextNode->getObstacle(j)->_unitDir;
					pDir = (portal->getLeft() - portal->getRight());
					isLeft = false;
				}
				else
				{
					continue;
				}
				pDir.Normalize();
				auto dot = oDir.Dot(pDir);
				auto oLen = oDir.Length();
				auto pLen = pDir.Length();
				auto magn = oLen * pLen;
				auto cos = dot / magn;
				auto sin = sqrt(1 - cos * cos);

				if (isLeft)
					leftOffset = std::max(leftOffset, agentRadius / sin);
				else
					rightOffset = std::max(rightOffset, agentRadius / sin);
			}
			/*Vector2 dir;
			if (i <= 1)
			{
				dir = path->getWayPoint(i) - path->getWayPoint(i - 1);
			}
			if (i > 1)
			{
				dir = path->getWayPoint(i) - path->getWayPoint(i - 1);
			}
			if (i > 1)
			{
				dir = path->getWayPoint(i) - path->getWayPoint(i - 1);
				float a = 30;
				float angle = a;
				auto rCos = std::cos(angle * std::_Pi / 180.0f);
				auto rSin = std::sin(angle * std::_Pi / 180.0f);
				auto dirLeft = Vector2(dir.x * rCos - dir.y * rSin, dir.x * rSin + dir.y * rCos);
				auto dirLeftNormal = dirLeft;
				dirLeftNormal.Normalize();

				angle = 360 - a;
				rCos = std::cos(angle * std::_Pi / 180.0f);
				rSin = std::sin(angle * std::_Pi / 180.0f);
				auto dirRight = Vector2(dir.x * rCos - dir.y * rSin, dir.x * rSin + dir.y * rCos);
				auto dirRightNormal = dirRight;
				dirRightNormal.Normalize();

				auto leftPoint = path->getPortal(i)->intersectionPoint(path->getWayPoint(i - 1)+ dirLeft, dirLeftNormal);
				auto rightPoint = path->getPortal(i)->intersectionPoint(path->getWayPoint(i - 1) + dirRight, dirRightNormal);
				auto t = 0;
			}*/



			float width = Vector2::Distance(portal->getLeft(), portal->getRight());
			auto old_wp = portal->getLeft(width * customEdgePosition);
			Vector2 delta = (portal->getLeft() - portal->getRight()) / 2;
			std::uniform_real_distribution<float> dist(-customDistribution, customDistribution);
			delta *= dist(_rnd_seed);
			auto newWaypoint = old_wp + delta;
			float newToLeft = Vector2::Distance(portal->getLeft(), newWaypoint);
			float newToRight = Vector2::Distance(newWaypoint, portal->getRight());
			newWaypoint =
				newToLeft > (width - rightOffset) ? portal->getRight(rightOffset) :
				newToRight > (width - leftOffset) ? portal->getLeft(leftOffset) :
				newWaypoint;

			path->setWaypoints(i, i + 1, newWaypoint, path->_headings[i]);
		}
	}
}