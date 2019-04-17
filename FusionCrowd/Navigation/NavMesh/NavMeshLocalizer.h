#pragma once

#include "NavMesh.h"
#include "NavMeshNode.h"
#include "Resource.h"
#include "Agent.h"
#include "Config.h"
#include "Math/Util.h"
#include <unordered_map>


#include <map>
#include <set>

namespace Agents {
	class BaseAgent;
}

// Forward declaration
class PortalPath;
class PathPlanner;

class FUSION_CROWD_API NavMeshLocation
{
public:
	NavMeshLocation() :_nodeID(NO_NODE), _hasPath(false) {}
	NavMeshLocation(unsigned int nodeID) :_nodeID(nodeID), _hasPath(false) {}
	NavMeshLocation(PortalPath * path) :_path(path), _hasPath(true) {}
	void setNode(unsigned int nodeID);
	void clearPath();
	unsigned int getNode() const;
	void setPath(PortalPath * path);
	inline bool isPath() const { return _hasPath; }
	inline bool isNode() const { return !_hasPath; }
	union {
		size_t _nodeID;
		PortalPath * _path;
	};
	bool	_hasPath;
	const static unsigned int NO_NODE = std::numeric_limits< unsigned int >::max();
};

typedef std::set< size_t > OccupantSet;
typedef OccupantSet::iterator OccupantSetItr;
typedef OccupantSet::const_iterator OccupantSetCItr;


class FUSION_CROWD_API NavMeshLocalizer : public Resource
{
public:
	NavMeshLocalizer(const std::string & name);
	~NavMeshLocalizer();
	virtual const std::string & getLabel() const { return LABEL; }
	unsigned int getNode(const FusionCrowd::Agent * agent) const;
	unsigned int getNode(const FusionCrowd::Agent * agent, const std::string & grpName,
		bool searchAll = false);
	unsigned int getNode(const DirectX::SimpleMath::Vector2 & p) const;
	const NavMeshNode getNode(unsigned int i) { return _navMesh->GetNode(i); }
	PortalPath * getPath(size_t id);
	void setPath(size_t agentID, PortalPath * path);
	void clearPath(size_t agentID);
	void setNode(size_t agentID, unsigned int nodeID);
	void setTrackAll() { _trackAll = true; }
	unsigned int updateLocation(const FusionCrowd::Agent * agent, bool force = false) const;
	void setPlanner(PathPlanner * planner) { _planner = planner; }
	PathPlanner * getPlanner() { return _planner; }
	const OccupantSet * getNodeOccupants(unsigned int nodeID) const {
		return &_nodeOccupants[nodeID];
	}
	const NavMeshPtr getNavMesh() const { return _navMesh; }
	NavMeshPtr getNavMesh() { return _navMesh; }
	static Resource * load(const std::string & fileName);
	static constexpr const char* LABEL = "navmesh_localizer";

	friend class PortalPath;

	NavMeshPtr _navMesh;
	bool _trackAll;
	PathPlanner * _planner;
	mutable  std::unordered_map< size_t, NavMeshLocation > _locations;

	OccupantSet * _nodeOccupants;
	unsigned int findNodeBlind(const DirectX::SimpleMath::Vector2 & p, float tgtElev = 1e5f) const;
	unsigned int findNodeInGroup(const DirectX::SimpleMath::Vector2 & p, const std::string & grpName,
		bool searchAll) const;
	unsigned int findNodeInRange(const DirectX::SimpleMath::Vector2 & p, unsigned int start,
		unsigned int stop) const;
	unsigned int testNeighbors(const NavMeshNode & node, const DirectX::SimpleMath::Vector2 & p) const;

};

typedef ResourcePtr< NavMeshLocalizer > NavMeshLocalizerPtr;

NavMeshLocalizerPtr FUSION_CROWD_API loadNavMeshLocalizer(const std::string & fileName,
	bool usePlanner);

