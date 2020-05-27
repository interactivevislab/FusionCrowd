#pragma once

#include "NavMesh.h"
#include "NavMeshNode.h"
#include "Agent.h"
#include "Math/Util.h"
#include "Navigation/NavMesh/QuadTree.h"

#include <set>

namespace FusionCrowd
{
	class BaseAgent;

	// Forward declaration
	class PortalPath;
	class PathPlanner;

	class NavMeshLocation
	{
	public:
		NavMeshLocation() : _nodeID(NO_NODE), _hasPath(false)
		{
		}

		NavMeshLocation(unsigned int nodeID) : _nodeID(nodeID), _hasPath(false)
		{
		}

		NavMeshLocation(std::shared_ptr<PortalPath> path) : _path(path), _hasPath(true)
		{
		}

		~NavMeshLocation()
		{
		}

		void setNode(unsigned int nodeID);
		void clearPath();
		unsigned int getNode() const;
		void setPath(std::shared_ptr<PortalPath> path);
		std::shared_ptr<PortalPath> getPath();
		inline bool isPath() const { return _hasPath; }
		inline bool isNode() const { return !_hasPath; }

		size_t _nodeID;
		std::shared_ptr<PortalPath> _path;

		bool _hasPath;
		const static unsigned int NO_NODE = std::numeric_limits<unsigned int>::max();
	};

	typedef std::set<size_t> OccupantSet;
	typedef OccupantSet::iterator OccupantSetItr;
	typedef OccupantSet::const_iterator OccupantSetCItr;


	class NavMeshLocalizer
	{
	public:
		NavMeshLocalizer(const std::string& fileName, bool usePlanner);
		~NavMeshLocalizer();

		unsigned int getNodeId(const DirectX::SimpleMath::Vector2& p) const;

		void setTrackAll() { _trackAll = true; }

		std::shared_ptr<PathPlanner> getPlanner() { return _planner; }
		void setPlanner(std::shared_ptr<PathPlanner> planner) { _planner = planner; }

		const std::shared_ptr<NavMesh> getNavMesh() const { return _navMesh; }

		std::vector<size_t> findNodesCrossingBB(BoundingBox bb);
		unsigned int findNodeBlind(const DirectX::SimpleMath::Vector2& p, float tgtElev = 1e5f) const;
		unsigned int findNodeInGroup(const DirectX::SimpleMath::Vector2& p, const std::string& grpName, bool searchAll) const;
		unsigned int findNodeInRange(const DirectX::SimpleMath::Vector2& p, unsigned int start, unsigned int stop) const;
		unsigned int testNeighbors(const NavMeshNode& node, const DirectX::SimpleMath::Vector2& p) const;
		DirectX::SimpleMath::Vector2 GetClosestAvailablePoint(DirectX::SimpleMath::Vector2 p);
		void Update(std::vector<NavMeshNode*>& added_nodes, std::vector<size_t>& del_nodes);

	private:
		std::shared_ptr<PathPlanner> _planner;
		std::shared_ptr<NavMesh> _navMesh;
		bool _trackAll;
		std::unique_ptr<QuadTree> _nodeBBTree;
	};
}
