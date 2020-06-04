#include "Route.h"

namespace FusionCrowd
{
	PortalRoute::PortalRoute(unsigned int start, unsigned int end, size_t navMeshVersion) :
		_startNode(start), _endNode(end), _maxWidth(1e6f), _bestSmallest(1e6f), _length(0.f), _nmVersion(navMeshVersion)
	{
	}

	bool PortalRoute::IsValid(size_t navMeshVersion) const
	{
		return navMeshVersion == _nmVersion;
	}

	void PortalRoute::appendWayPortal(const NavMeshEdge* edge, unsigned int node)
	{
		_length += edge->getNodeDistance();
		float w = edge->getWidth();
		_maxWidth = w < _maxWidth ? w : _maxWidth;
		_portals.push_back(WayPortal(edge, node, edge->pointOnLeft(node)));
	}

	bool PortalRoute::isEquivalent(const PortalRoute* route)
	{
		const size_t PORTAL_COUNT = _portals.size();
		if (PORTAL_COUNT == route->_portals.size())
		{
			for (size_t i = 0; i < PORTAL_COUNT; ++i)
			{
				if (_portals[i]._nodeID != route->_portals[i]._nodeID)
				{
					break;
				}
			}
			return true;
		}
		return false;
	}
}
