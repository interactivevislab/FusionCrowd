#include "Funnel.h"
#include "PortalPath.h"

#include "PathRandomization.h"

#include <cassert>
#include <iostream>

namespace FusionCrowd
{
	FunnelPlanner::FunnelPlanner()
	{
	}

	FunnelPlanner::~FunnelPlanner()
	{
	}

	using namespace DirectX::SimpleMath;

	void FunnelPlanner::computeCrossing(float radius, const Vector2& startPos, FusionCrowd::PortalPath* path,
	                                    size_t startPortal)
	{
		assert(path->getPortalCount() > 0 &&
			"Funnel planner should only be applied to PortalPaths with at least one portal");
		// if startPortal is zero, this should go to all 1s...i.e. -1 > all other size_t values
		FunnelApex apex(startPortal - 1, startPos);

		const FusionCrowd::WayPortal* portal = path->getPortal(startPortal);
		Vector2 pLeft(portal->getLeft(radius));
		Vector2 pRight(portal->getRight(radius));
		Vector2 dirLeft(pLeft - apex._pos);
		Vector2 dirRight(pRight - apex._pos);
#ifdef SIMPLE_FUNNEL
	FunnelEdge funnelLeft(startPortal, dirLeft);
	FunnelEdge funnelRight(startPortal, dirRight);
	size_t currPortal = startPortal + 1;
	const size_t PORTAL_COUNT = path->getPortalCount();
	while (currPortal < PORTAL_COUNT) {
		portal = path->getPortal(currPortal);
		pLeft.set(portal->getLeft(radius));
		pRight.set(portal->getRight(radius));
		dirLeft.set(pLeft - apex._pos);
		dirRight.set(pRight - apex._pos);

		// test left side of the funnel
		if (funnelRight.isOnRight(dirLeft)) {
			// the portal's funnel is on the right of the current funnel
			Vector2 oldApex = apex._pos;
			Vector2 newApex = funnelRight._dir + apex._pos;
			//if ( apex._id != -1 && apex._id < currPortal ) {
			path->setWaypoints(apex._id + 1, funnelRight._id + 1, newApex,
				norm(funnelRight._dir));
			//}
			apex.set(funnelRight._id, newApex);
			currPortal = funnelRight._id + 1;

			portal = path->getPortal(currPortal);
			pLeft.set(portal->getLeft(radius));
			pRight.set(portal->getRight(radius));
			dirLeft.set(pLeft - apex._pos);
			dirRight.set(pRight - apex._pos);
			funnelLeft.set(currPortal, dirLeft);
			funnelRight.set(currPortal, dirRight);
			++currPortal;

			continue;
		}
		else if (funnelLeft.isOnRight(dirLeft)) {
			funnelLeft.set(currPortal, dirLeft);
		}

		// test right side of the funnel
		if (funnelLeft.isOnLeft(dirRight)) {
			// the portal's funnel is on the left of the current funnel
			Vector2 oldApex = apex._pos;
			Vector2 newApex = funnelLeft._dir + apex._pos;
			//if ( apex._id != -1 && apex._id < currPortal ) {
			path->setWaypoints(apex._id + 1, funnelLeft._id + 1, newApex,
				norm(funnelLeft._dir));
			//}
			apex.set(funnelLeft._id, newApex);
			currPortal = funnelLeft._id + 1;

			portal = path->getPortal(currPortal);
			pLeft.set(portal->getLeft(radius));
			pRight.set(portal->getRight(radius));
			dirLeft.set(pLeft - apex._pos);
			dirRight.set(pRight - apex._pos);
			funnelLeft.set(currPortal, dirLeft);
			funnelRight.set(currPortal, dirRight);
			++currPortal;

			continue;
		}
		else if (funnelRight.isOnLeft(dirRight)) {
			funnelRight.set(currPortal, dirRight);
		}
		++currPortal;
	}

	// Now handle goal
	const Vector2 goalPt = path->getGoalCentroid();
	Vector2 goalDir(goalPt - apex._pos);

	if (funnelLeft.isOnLeft(goalDir)) {
		// The goal point is on the left side of the funnel
		if (apex._id != -1 && apex._id < PORTAL_COUNT) {
			path->setWaypoints(apex._id + 1, PORTAL_COUNT, apex._pos + funnelLeft._dir,
				norm(funnelLeft._dir));
		}
	}
	else if (funnelRight.isOnRight(goalDir)) {
		// The goal point is on the right side of the funnel
		if (apex._id != -1 && apex._id < PORTAL_COUNT) {
			path->setWaypoints(apex._id + 1, PORTAL_COUNT, apex._pos + funnelRight._dir,
				norm(funnelRight._dir));
		}
	}

	if (apex._id + 1 < PORTAL_COUNT) {
		path->setWaypoints((size_t)apex._id + 1, (size_t)PORTAL_COUNT, goalPt,
			norm(goalPt - apex._pos));
	}

#else
		_left.push_back(FunnelEdge(startPortal - 1, startPortal, dirLeft, startPos));
		_right.push_back(FunnelEdge(startPortal - 1, startPortal, dirRight, startPos));
		const size_t PORTAL_COUNT = path->getPortalCount();
		for (size_t i = startPortal + 1; i < PORTAL_COUNT; ++i)
		{
			portal = path->getPortal(i);

			// investigate the left point
			pLeft = portal->getLeft(radius);
			bool apexMoved = false;
			while (!_right.empty())
			{
				std::list<FunnelEdge>::iterator itr = _right.begin();
				Vector2 dir = pLeft - itr->_origin;
				if (itr->isOnRight(dir))
				{
					apexMoved = true;
					Vector2 newApex = itr->_origin + itr->_dir;
					apex.set(itr->_endID, newApex);

					Vector2 normDir;
					itr->_dir.Normalize(normDir);
					path->setWaypoints(itr->_id + 1, itr->_endID + 1, newApex, normDir);

					_right.pop_front();
				}
				else
				{
					break;
				}
			}
			if (apexMoved)
			{
				_left.clear();
				_left.push_back(FunnelEdge(apex._id, i, pLeft - apex._pos, apex._pos));
			}
			else
			{
				std::list<FunnelEdge>::reverse_iterator itr = _left.rbegin();
				while (!_left.empty())
				{
					Vector2 dir = pLeft - itr->_origin;
					if (itr->isOnRight(dir))
					{
						_left.pop_back();
						itr = _left.rbegin();
					}
					else
					{
						break;
					}
				}
				if (_left.empty())
				{
					_left.push_back(FunnelEdge(apex._id, i, pLeft - apex._pos, apex._pos));
				}
				else
				{
					Vector2 origin(itr->_origin + itr->_dir);
					_left.push_back(FunnelEdge(itr->_endID, i, pLeft - origin, origin));
				}
			}

			// investigate the right point
			pRight = portal->getRight(radius);
			apexMoved = false;
			while (!_left.empty())
			{
				std::list<FunnelEdge>::iterator itr = _left.begin();
				Vector2 dir = pRight - itr->_origin;
				if (itr->isOnLeft(dir))
				{
					apexMoved = true;
					Vector2 newApex = itr->_origin + itr->_dir;
					Vector2 normDir;
					itr->_dir.Normalize(normDir);
					path->setWaypoints(itr->_id + 1, itr->_endID + 1, newApex, normDir);
					apex.set(itr->_endID, newApex);
					_left.pop_front();
				}
				else
				{
					break;
				}
			}
			if (apexMoved)
			{
				_right.clear();
				_right.push_back(FunnelEdge(apex._id, i, pRight - apex._pos, apex._pos));
			}
			else
			{
				std::list<FunnelEdge>::reverse_iterator itr = _right.rbegin();
				while (!_right.empty())
				{
					Vector2 dir = pRight - itr->_origin;
					if (itr->isOnLeft(dir))
					{
						_right.pop_back();
						itr = _right.rbegin();
					}
					else
					{
						break;
					}
				}
				if (_right.empty())
				{
					_right.push_back(FunnelEdge(apex._id, i, pRight - apex._pos, apex._pos));
				}
				else
				{
					Vector2 origin(itr->_origin + itr->_dir);
					_right.push_back(FunnelEdge(itr->_endID, i, pRight - origin, origin));
				}
			}
		}
		// handle the goal
		const Vector2 goalPt = path->getGoalCentroid();
		Vector2 goalDir;

		bool apexMoved = false;
		while (!_left.empty())
		{
			std::list<FunnelEdge>::iterator itr = _left.begin();
			goalDir = goalPt - itr->_origin;
			if (itr->isOnLeft(goalDir))
			{
				apexMoved = true;
				Vector2 newApex = itr->_origin + itr->_dir;
				apex.set(itr->_endID, newApex);

				Vector2 normDir;
				itr->_dir.Normalize(normDir);
				path->setWaypoints(itr->_id + 1, itr->_endID + 1, newApex, normDir);
				_left.pop_front();
			}
			else
			{
				break;
			}
		}
		if (apexMoved)
		{
			(goalPt - apex._pos).Normalize(goalDir);
			path->setWaypoints(apex._id + 1, PORTAL_COUNT, goalPt, goalDir);
		}
		else
		{
			// apexMoved is already false -- it is the only way to reach this branch
			while (!_right.empty())
			{
				std::list<FunnelEdge>::iterator itr = _right.begin();
				goalDir = goalPt - itr->_origin;
				if (itr->isOnRight(goalDir))
				{
					apexMoved = true;
					Vector2 newApex = itr->_origin + itr->_dir;
					apex.set(itr->_endID, newApex);

					Vector2 normDir;
					itr->_dir.Normalize(normDir);
					path->setWaypoints(itr->_id + 1, itr->_endID + 1, newApex, normDir);
					_right.pop_front();
				}
				else
				{
					break;
				}
			}

			(goalPt - apex._pos).Normalize(goalDir);

			path->setWaypoints(apex._id + 1, PORTAL_COUNT, goalPt, goalDir);
		}
		auto pr = PathRandomization();
		pr.RandomizePath(path);
#endif	// SIMPLE_FUNNEL
	}
}
