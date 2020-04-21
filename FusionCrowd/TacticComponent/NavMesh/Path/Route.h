#pragma once

#include "WayPortal.h"

#include <vector>

namespace FusionCrowd
{
	// FORWARD DECLARATIONS
	class PathPlanner;
	class NavMeshEdge;

	class PortalRoute
	{
	public:
		PortalRoute(unsigned int start, unsigned int end, size_t navMeshVersion);

		inline unsigned int getStartNode() const { return _startNode; }
		inline unsigned int getEndNode() const { return _endNode; }
		inline size_t getPortalCount() const { return _portals.size(); }
		inline unsigned int getPortalNode(size_t i) const { return _portals[i]._nodeID; }
		WayPortal* getPortal(size_t i) { return &_portals[i]; }
		const WayPortal* getPortal(size_t i) const { return &_portals[i]; }
		void appendWayPortal(const NavMeshEdge* edge, unsigned int node);
		bool isEquivalent(const PortalRoute* route);

		bool IsValid(size_t navMeshVersion) const;

		friend class PathPlanner;

	protected:
		unsigned int _startNode;
		unsigned int _endNode;
		float _maxWidth;
		float _bestSmallest;
		float _length;
		std::vector<WayPortal> _portals;
		size_t _nmVersion;
	};
}
