#pragma once

#include "WayPortal.h"
#include "../Config.h"

#include <vector>

// FORWARD DECLARATIONS
class PathPlanner;
class NavMeshEdge;

class FUSION_CROWD_API PortalRoute
{
public:
	PortalRoute(unsigned int start, unsigned int end);
	~PortalRoute();
	inline unsigned int getStartNode() const { return _startNode; }
	inline unsigned int getEndNode() const { return _endNode; }
	inline size_t getPortalCount() const { return _portals.size(); }
	inline unsigned int getPortalNode(size_t i) const { return _portals[i]._nodeID; }
	WayPortal * getPortal(size_t i) { return &_portals[i]; }
	const WayPortal * getPortal(size_t i) const { return &_portals[i]; }
	void appendWayPortal(const NavMeshEdge * edge, unsigned int node);
	bool isEquivalent(const PortalRoute * route);
	friend class PathPlanner;

protected:
	unsigned int	_startNode;
	unsigned int	_endNode;
	float _maxWidth;
	float _bestSmallest;
	float _length;
	std::vector< WayPortal >	_portals;
};