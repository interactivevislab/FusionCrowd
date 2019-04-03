#pragma once

#include "../NavComponents/NavMesh/NavMesh.h"
#include "../Math/vector.h"
#include "../Config.h"

#include <list>
#include <map>
#include <unordered_map>


// FORWARD DECLARATIONS
class PortalRoute;
class PathPlanner;

typedef size_t RouteKey;
typedef std::list< PortalRoute * > PRouteList;
typedef PRouteList::iterator PRouteListItr;
typedef PRouteList::const_iterator PRouteListCItr;
typedef std::unordered_map< RouteKey, PRouteList > PRouteMap;
typedef PRouteMap::iterator PRouteMapItr;
typedef PRouteMap::const_iterator PRouteMapCItr;

class FUSION_CROWD_API PathPlanner
{
public:
	PathPlanner(NavMeshPtr ptr);
	~PathPlanner();
	PortalRoute * getRoute(unsigned int startID, unsigned int endID, float minWidth);
protected:
	PortalRoute * computeRoute(unsigned int startID, unsigned int endID, float minWidth);
	float computeH(unsigned int node, const FusionCrowd::Math::Vector2 & goal);
	PortalRoute * cacheRoute(unsigned int startID, unsigned int endID, PortalRoute * route);
	PRouteMap _routes;
	NavMeshPtr	_navMesh;
	void initHeapMemory(size_t nodeCount);
	size_t DATA_SIZE;
	size_t STATE_SIZE;
	unsigned int * _HEAP;
	unsigned int * _PATH;
	float * _DATA;
	bool *	_STATE;
};

