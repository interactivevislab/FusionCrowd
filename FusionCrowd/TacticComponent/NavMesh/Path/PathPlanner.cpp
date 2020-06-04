#include "PathPlanner.h"
#include "MinHeap.h"
#include "Route.h"

#include "Navigation/NavMesh/NavMesh.h"
#include "Navigation/NavMesh/NavMeshNode.h"

#include <iostream>
#include <cassert>
#include <sstream>
#include "Navigation/NavMesh/NavMeshLocalizer.h"

#ifdef _OPENMP
#include <omp.h>
#endif

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	RouteKey makeRouteKey(unsigned int start, unsigned int end)
	{
		const int SHIFT = sizeof(size_t) * 4; // size in bytes * 8 bits/byte / 2
		const size_t MASK = (1UL << SHIFT) - 1;
		assert((size_t)start == ((size_t)start & MASK));
		assert((size_t)end == ((size_t)end & MASK));
		return ((size_t)start << SHIFT) | ((size_t)end & MASK);
	}

	PathPlanner::PathPlanner(std::shared_ptr<NavMesh> ptr) : _navMesh(ptr), DATA_SIZE(0), STATE_SIZE(0),
	                                           _HEAP(0x0), _DATA(0x0), _STATE(0x0)
	{
		size_t nCount = _navMesh->getNodeCount();
		initHeapMemory(nCount);
	}


	PathPlanner::~PathPlanner()
	{
		initHeapMemory(0);
	}

	PortalRoute* PathPlanner::getRoute(unsigned int startID, unsigned int endID,
	                                   float minWidth)
	{
		RouteKey key = makeRouteKey(startID, endID);

		PortalRoute* route = NULL;
		//_routeLock.lockRead();
		PRouteMapItr itr = _routes.find(key);
		if (itr != _routes.end())
		{
			// test the routes to see if they are passable
			PRouteListItr rItr = itr->second.begin();
			for (; rItr != itr->second.end(); ++rItr)
			{
				if ((*rItr)->_maxWidth > minWidth)
				{
					if ((*rItr)->_bestSmallest <= minWidth * 1.05f)
					{
						route = *rItr;
					}
				}
			}
		}
		//_routeLock.releaseRead();

		// Compute a new path
		if (route == 0x0)
		{
			return computeRoute(startID, endID, minWidth);
		}
		else
		{
			return route;
		}
	}

	PortalRoute* PathPlanner::computeRoute(unsigned int startID, unsigned int endID
	                                       , float minWidth)
	{
		const size_t N = _navMesh->getNodeCount();
#ifdef _OPENMP
	// Assuming that threadNum \in [0, omp_get_max_threads() )
	const unsigned int threadNum = omp_get_thread_num();
	AStarMinHeap heap(_HEAP + threadNum * N, _DATA + threadNum * DATA_SIZE,
		_STATE + threadNum * STATE_SIZE, _PATH + threadNum * N, N);
#else

		AStarMinHeap heap(_HEAP, _DATA, _STATE, _PATH, N);
#endif

		const Vector2 goalPos(_navMesh->GetNodeByPos(endID).getCenter());

		heap.g(startID, 0);
		heap.h(startID, computeH(startID, goalPos));
		heap.f(startID, heap.h(startID));
		heap.push(startID);

		bool found = false;
		while (!heap.empty())
		{
			unsigned int x = heap.pop();

			if (x == endID)
			{
				found = true;
				break;
			}
			NavMeshNode& node = _navMesh->GetNodeByPos(x);
			for (size_t e = 0; e < node._edgeCount; ++e)
			{
				NavMeshEdge* edge = node._edges[e];
				unsigned int y = edge->getOtherByID(x)->_id;
				if (heap.isVisited(y)) continue;
				float distance = edge->getNodeDistance(minWidth);
				if (distance < 0.f) continue;
				float tempG = heap.g(x) + distance;

				bool isOld = true;
				if (!heap.isInHeap(y))
				{
					heap.h(y, computeH(y, goalPos));
					isOld = false;
				}
				if (tempG < heap.g(y))
				{
					heap.setReachedFrom(y, x);
					heap.g(y, tempG);
					heap.f(y, tempG + heap.h(y));
				}
				if (!heap.isInHeap(y))
				{
					heap.push(y);
				}
			}
		}

		std::list<unsigned int> path;
		// reconstruct the path
		if (!found)
		{
			std::stringstream ss;
			ss << "Trying to find a path from " << startID << " to " << endID;
			ss << ".  A* finished without a route!";

			path.push_front(startID);

		}
		else {
			// Create the list of nodes through which I must pass
			unsigned int curr = endID;
			while (curr != startID)
			{
				path.push_front(curr);
				curr = heap.getReachedFrom(curr);
			}
			path.push_front(startID);
		}

#ifdef _WIN32
		// Visual studio 2005 compiler is giving an erroneous warning
		// It feels that:
		//		unsigned int prev = *itr;
		// Is trying to cast a value of type size_t into a value of type unsigned int
		// which, if true, would possibly lose data.  However, this is simply incorrect,
		// as the iterator is to a list of unsigned ints.
		//
		// Because the code is correct, this is the only way to silence the stupid warning.
#pragma warning( disable : 4267 )
#endif

		// Now construct the path
		std::list<unsigned int>::const_iterator itr = path.begin();
		unsigned int prev = *itr;
		NavMeshNode* prevNode = &_navMesh->GetNodeByPos(prev);
		++itr;

		PortalRoute* route = new PortalRoute(startID, endID, _navMesh->GetVersion());
		route->_bestSmallest = minWidth;
		for (; itr != path.end(); ++itr)
		{
			unsigned int id = *itr;
			NavMeshEdge* edge = prevNode->getConnection(id);
			route->appendWayPortal(edge, prevNode->getID());
			prevNode = &_navMesh->GetNodeByPos(id);
		}
#ifdef _WIN32
#pragma warning( default : 4267 )
#endif
		cacheRoute(startID, endID, route);
		return route;
	}

	void PathPlanner::initHeapMemory(size_t nodeCount)
	{
		int threadCount = 1;
#ifdef _OPENMP
	logger << Logger::INFO_MSG << "OMP ENABLED!\n";
	threadCount = omp_get_max_threads();
#endif
		if (_DATA)
		{
			delete[] _DATA;
			_DATA = 0x0;
			delete[] _STATE;
			_STATE = 0x0;
			delete[] _HEAP;
			_HEAP = 0x0;
			delete[] _PATH;
			_PATH = 0x0;
		}

		DATA_SIZE = 3 * nodeCount; // the number of floats per thread in _data
		STATE_SIZE = 2 * nodeCount; // the number of bools, per thread, in _state;
		if (nodeCount)
		{
			_DATA = new float[threadCount * DATA_SIZE];
			_STATE = new bool[threadCount * STATE_SIZE];
			_HEAP = new unsigned int[threadCount * nodeCount];
			_PATH = new unsigned int[threadCount * nodeCount];
		}
	}

	float PathPlanner::computeH(unsigned int node, const Vector2& goal)
	{
		assert(node >= 0 && node < _navMesh->getNodeCount() &&
			"Trying to compute h for invalid node id");
		return (_navMesh->GetNodeByPos(node)._center - goal).Length();
	}

	PortalRoute* PathPlanner::cacheRoute(unsigned int startID, unsigned int endID,
	                                     PortalRoute* route)
	{
		PortalRoute* result = route;
		RouteKey key = makeRouteKey(startID, endID);
		PRouteMapItr mapItr = _routes.find(key);
		if (mapItr == _routes.end())
		{
			// there have been no routes connecting these two points -- it is optimal
			_routes[key] = PRouteList();
			_routes[key].push_back(route);
		}
		else
		{
			// I have already found routes - are they equivalent?
			float w = route->_maxWidth;
			PRouteList& routeList = mapItr->second;
			PRouteListItr rItr = routeList.begin();
			for (; rItr != routeList.end(); ++rItr)
			{
				float rWidth = (*rItr)->_maxWidth;
				if (rWidth > w)
				{
					// The next width has the capacity to handle agents on this route
					// It is assumed that it hasn't ever been shown optimal for this route's
					//	required clearance (otherwise, we would've simply used it.
					//	Test to see if it is the same route
					if (route->isEquivalent((*rItr)))
					{
						result = *rItr;
						assert(route->_bestSmallest < result->_bestSmallest &&
							"Recomputed an equivalent path which was already shown to be "
							"sufficiently wide and optimal");
						result->_bestSmallest = route->_bestSmallest;
					}
					else
					{
						routeList.insert(rItr, route);
					}
					return route;
				}
			}
			routeList.push_back(route);
		}
		return route;
	}
}
