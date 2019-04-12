#include "NavMeshLocalizer.h"

#include "NavMeshNode.h"
#include "Path/PathPlanner.h"
#include "Path/PortalPath.h"

#include <limits>

using namespace DirectX::SimpleMath;

void NavMeshLocation::setNode(unsigned int nodeID)
{
	if (_hasPath) {
		delete _path;
		_hasPath = false;
	}
	_nodeID = nodeID;
}

void NavMeshLocation::clearPath()
{
	if (_hasPath) {
		unsigned int node = _path->getNode();
		delete _path;
		_hasPath = false;
		_nodeID = (size_t)node;
	}
}

unsigned int NavMeshLocation::getNode() const {
	//TODO: NavMesh nodes should simply be size_ts and NOT unsigned ints.
	if (_hasPath) {
		return (unsigned int)_path->getNode();
	}
	else {
		return (unsigned int)_nodeID;
	}
}

void NavMeshLocation::setPath(PortalPath * path) {
	if (_hasPath) {
		delete _path;
	}
	_path = path;
	_hasPath = true;
}

//const std::string NavMeshLocalizer::LABEL ="navmesh_localizer";

NavMeshLocalizer::NavMeshLocalizer(const std::string & name) : Resource(name), _navMesh(0x0),
_trackAll(false), _planner(0x0)
{
	_navMesh = loadNavMesh(name);

	const size_t NODE_COUNT = _navMesh->getNodeCount();
	_nodeOccupants = new OccupantSet[NODE_COUNT + 1];
}

NavMeshLocalizer::~NavMeshLocalizer()
{
	delete[] _nodeOccupants;
}

void NavMeshLocalizer::clearPath(size_t agentID)
{
	if (_locations.count(agentID) > 0) {
		_locations[agentID].clearPath();
	}
}

unsigned int NavMeshLocalizer::getNode(const FusionCrowd::Agent * agent) const
{
	unsigned int node = NavMeshLocation::NO_NODE;
	if (_locations.count(agent->_id) > 0) {
		node = _locations[agent->_id].getNode();
	}
	return node;
}

unsigned int NavMeshLocalizer::getNode(const FusionCrowd::Agent * agent,
	const std::string & grpName, bool searchAll)
{
	unsigned int node = getNode(agent);
	if (node == NavMeshLocation::NO_NODE) {
		//node = findNodeInGroup(agent->_pos, grpName, searchAll);
		if (node != NavMeshLocation::NO_NODE) {
			setNode(agent->_id, node);
		}
	}
	return node;
}

unsigned int NavMeshLocalizer::getNode(const Vector2 & p) const
{
	return findNodeBlind(p);
}

PortalPath * NavMeshLocalizer::getPath(size_t id)
{
	PortalPath * path = 0x0;
	int testLocCount = _locations.count(id);
	if (_locations.count(id) > 0) {
		if (_locations[id].isPath()) {
			path = _locations[id]._path;
		}
	}
	return path;
}

void NavMeshLocalizer::setPath(size_t agentID, PortalPath * path)
{
	_locations[agentID].setPath(path);
}

void NavMeshLocalizer::setNode(size_t agentID, unsigned int nodeID)
{
	_locations[agentID].setNode(nodeID);
}

unsigned int NavMeshLocalizer::updateLocation(const FusionCrowd::Agent * agent,
	bool force) const
{
	const size_t ID = agent->_id;
	// NOTE: This will create a default location instance if the agent didn't already
	//	have one
	NavMeshLocation & loc = _locations[ID];

	unsigned int oldLoc = loc.getNode();
	unsigned int newLoc = oldLoc;
	if (loc._hasPath) {
		newLoc = loc._path->updateLocation(agent, _navMesh, this, _planner);
	}
	else { //if ( _trackAll || force ) {
		const Vector2 & p = agent->_pos;
		unsigned int oldNode = (unsigned int)loc._nodeID;
		if (loc._nodeID == NavMeshLocation::NO_NODE) {
			loc._nodeID = findNodeBlind(p);
		}
		else {
			const NavMeshNode & node = _navMesh->GetNode((unsigned int)loc._nodeID);
			if (!node.containsPoint(p)) {	// not in current node
				loc._nodeID = testNeighbors(node, p);
				if (loc._nodeID == NavMeshLocation::NO_NODE) {
					loc._nodeID = findNodeBlind(p);
				}
			}
		}
		if (loc._nodeID == NavMeshLocation::NO_NODE) {
			loc._nodeID = oldNode;
		}
		newLoc = (unsigned int)loc._nodeID;
	}

	if (newLoc != oldLoc) {
		if (newLoc == NavMeshLocation::NO_NODE) {
			newLoc = static_cast<unsigned int>(_navMesh->getNodeCount());
		}

		// remove the agent from the set for oldLoc and place it in newLoc
#pragma omp critical( NAV_MESH_LOCALIZER_MOVE_AGENT )
		{
			if (oldLoc != NavMeshLocation::NO_NODE) {
				OccupantSetItr fromItr = _nodeOccupants[oldLoc].find(ID);
				if (fromItr != _nodeOccupants[oldLoc].end()) {
					_nodeOccupants[oldLoc].erase(fromItr);
				}
				else if (oldLoc != NavMeshLocation::NO_NODE) {
					const size_t NCOUNT = _navMesh->getNodeCount();
					for (size_t i = 0; i < NCOUNT; ++i) {
						fromItr = _nodeOccupants[i].find(ID);
						if (fromItr != _nodeOccupants[i].end()) {
							_nodeOccupants[i].erase(fromItr);
							break;
						}
					}
				}
			}
			_nodeOccupants[newLoc].insert(ID);
		}
	}

	return newLoc;
}

unsigned int NavMeshLocalizer::findNodeBlind(const Vector2 & p, float tgtElev) const
{
	// TODO(curds01) 10/1/2016 - This cast is bad because I can lose precision
	//	(after I get 4 billion nodes...)
	const unsigned int nCount = static_cast<unsigned int>(_navMesh->getNodeCount());
	float elevDiff = 1e6f;
	unsigned int maxNode = NavMeshLocation::NO_NODE;
	for (unsigned int n = 0; n < nCount; ++n) {
		const NavMeshNode & node = _navMesh->GetNode(n);
		if (node.containsPoint(p)) {
			float hDiff = fabs(node.getElevation(p) - tgtElev);
			if (hDiff < elevDiff) {
				maxNode = n;
				elevDiff = hDiff;
			}
		}
	}
	return maxNode;
}

unsigned int NavMeshLocalizer::findNodeInGroup(const Vector2 & p, const std::string & grpName,
	bool searchAll) const {
	unsigned int node = NavMeshLocation::NO_NODE;
	const NMNodeGroup * grp = _navMesh->getNodeGroup(grpName);
	if (grp != 0x0) {
		node = findNodeInRange(p, grp->_first, grp->_last + 1);

		if (node == NavMeshLocation::NO_NODE && searchAll) {
			node = findNodeInRange(p, 0, grp->_first);
			if (node == NavMeshLocation::NO_NODE) {
				// TODO(curds01) 10/1/2016 - This cast is bad because I can lose precision
				// (after I get 4 billion nodes...)
				const unsigned int TOTAL_NODES =
					static_cast<unsigned int>(_navMesh->getNodeCount());
				node = findNodeInRange(p, grp->_first + 1, TOTAL_NODES);
			}
		}
	}
	return node;
}

unsigned int NavMeshLocalizer::findNodeInRange(const Vector2 & p, unsigned int start,
	unsigned int stop) const
{
	for (unsigned int n = start; n < stop; ++n) {
		const NavMeshNode & node = _navMesh->GetNode(n);
		if (node.containsPoint(p)) {
			return n;
		}
	}
	return NavMeshLocation::NO_NODE;
}

unsigned int NavMeshLocalizer::testNeighbors(const NavMeshNode & node,
	const Vector2 & p) const
{
	const unsigned int nCount = static_cast<unsigned int>(node.getNeighborCount());
	for (unsigned int n = 0; n < nCount; ++n) {
		const NavMeshNode * nbr = node.getNeighbor(n);
		if (nbr->containsPoint(p)) {
			return nbr->getID();
		}
	}
	return NavMeshLocation::NO_NODE;
}

Resource * NavMeshLocalizer::load(const std::string & fileName) {
	NavMeshPtr mesh;

	mesh = loadNavMesh(fileName);

	NavMeshLocalizer * nml = new NavMeshLocalizer(fileName);
	nml->_navMesh = mesh;
	return nml;
}

NavMeshLocalizerPtr loadNavMeshLocalizer(const std::string & fileName, bool usePlanner)
{
	Resource * rsrc = ResourceManager::getResource(fileName,
		&NavMeshLocalizer::load,
		NavMeshLocalizer::LABEL);
	if (rsrc == 0x0)
	{
		return NULL;
	}
	NavMeshLocalizer * nml = dynamic_cast<NavMeshLocalizer *>(rsrc);
	if (nml == 0x0)
	{
		return NULL;
	}

	if (usePlanner) {
		if (nml->getPlanner() == 0x0) {
			PathPlanner * planner = new PathPlanner(nml->getNavMesh());
			nml->setPlanner(planner);
		}
	}

	return NavMeshLocalizerPtr(nml);
}