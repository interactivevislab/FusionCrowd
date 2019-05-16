#include "NavMeshLocalizer.h"

#include "NavMeshNode.h"
#include "TacticComponent/Path/PathPlanner.h"
#include "TacticComponent/Path/PortalPath.h"

#include <limits>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	void NavMeshLocation::setNode(unsigned int nodeID)
	{
		if (_hasPath)
		{
			delete _path;
			_hasPath = false;
		}
		_nodeID = nodeID;
	}

	void NavMeshLocation::clearPath()
	{
		if (_hasPath)
		{
			unsigned int node = _path->getNode();
			delete _path;
			_hasPath = false;
			_nodeID = (size_t)node;
		}
	}

	unsigned int NavMeshLocation::getNode() const
	{
		//TODO: NavMesh nodes should simply be size_ts and NOT unsigned ints.
		if (_hasPath)
		{
			return (unsigned int)_path->getNode();
		}
		else
		{
			return (unsigned int)_nodeID;
		}
	}

	void NavMeshLocation::setPath(PortalPath* path)
	{
		if (_hasPath)
		{
			delete _path;
		}
		_path = path;
		_hasPath = true;
	}

	PortalPath * NavMeshLocation::getPath()
	{
		if(_hasPath)
			return _path;
		else
			return nullptr;
	}

	//
	//

	void NavMeshLocalizer::updateAgentPosition(size_t agentId, const unsigned int oldLoc, unsigned int newLoc)
	{
		if (newLoc != oldLoc)
		{
			if (newLoc == NavMeshLocation::NO_NODE)
			{
				newLoc = static_cast<unsigned int>(_navMesh->getNodeCount());
			}

			// remove the agent from the set for oldLoc and place it in newLoc
#pragma omp critical( NAV_MESH_LOCALIZER_MOVE_AGENT )
			{
				if (oldLoc != NavMeshLocation::NO_NODE)
				{
					OccupantSetItr fromItr = _nodeOccupants[oldLoc].find(agentId);
					if (fromItr != _nodeOccupants[oldLoc].end())
					{
						_nodeOccupants[oldLoc].erase(fromItr);
					}
					else if (oldLoc != NavMeshLocation::NO_NODE)
					{
						const size_t NCOUNT = _navMesh->getNodeCount();
						for (size_t i = 0; i < NCOUNT; ++i)
						{
							fromItr = _nodeOccupants[i].find(agentId);
							if (fromItr != _nodeOccupants[i].end())
							{
								_nodeOccupants[i].erase(fromItr);
								break;
							}
						}
					}
				}
				_nodeOccupants[newLoc].insert(agentId);
			}
		}
	}

	//const std::string NavMeshLocalizer::LABEL ="navmesh_localizer";

	NavMeshLocalizer::NavMeshLocalizer(const std::string& name) : Resource(name), _navMesh(0x0),
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

	unsigned int NavMeshLocalizer::getNode(const Vector2& p) const
	{
		return findNodeBlind(p);
	}

	unsigned int NavMeshLocalizer::findNodeBlind(const Vector2& p, float tgtElev) const
	{
		// TODO(curds01) 10/1/2016 - This cast is bad because I can lose precision
		//	(after I get 4 billion nodes...)
		const unsigned int nCount = static_cast<unsigned int>(_navMesh->getNodeCount());
		float elevDiff = 1e6f;
		unsigned int maxNode = NavMeshLocation::NO_NODE;
		for (unsigned int n = 0; n < nCount; ++n)
		{
			const NavMeshNode& node = _navMesh->GetNode(n);
			if (node.containsPoint(p))
			{
				float hDiff = fabs(node.getElevation(p) - tgtElev);
				if (hDiff < elevDiff)
				{
					maxNode = n;
					elevDiff = hDiff;
				}
			}
		}
		return maxNode;
	}

	unsigned int NavMeshLocalizer::findNodeInGroup(const Vector2& p, const std::string& grpName, bool searchAll) const
	{
		unsigned int node = NavMeshLocation::NO_NODE;
		const NMNodeGroup* grp = _navMesh->getNodeGroup(grpName);
		if (grp != 0x0)
		{
			node = findNodeInRange(p, grp->_first, grp->_last + 1);

			if (node == NavMeshLocation::NO_NODE && searchAll)
			{
				node = findNodeInRange(p, 0, grp->_first);
				if (node == NavMeshLocation::NO_NODE)
				{
					// TODO(curds01) 10/1/2016 - This cast is bad because I can lose precision
					// (after I get 4 billion nodes...)
					const unsigned int TOTAL_NODES = static_cast<unsigned int>(_navMesh->getNodeCount());
					node = findNodeInRange(p, grp->_first + 1, TOTAL_NODES);
				}
			}
		}
		return node;
	}

	unsigned int NavMeshLocalizer::findNodeInRange(const Vector2& p, unsigned int start,
	                                               unsigned int stop) const
	{
		for (unsigned int n = start; n < stop; ++n)
		{
			const NavMeshNode& node = _navMesh->GetNode(n);
			if (node.containsPoint(p))
			{
				return n;
			}
		}
		return NavMeshLocation::NO_NODE;
	}

	unsigned int NavMeshLocalizer::testNeighbors(const NavMeshNode& node,
	                                             const Vector2& p) const
	{
		const unsigned int nCount = static_cast<unsigned int>(node.getNeighborCount());
		for (unsigned int n = 0; n < nCount; ++n)
		{
			const NavMeshNode* nbr = node.getNeighbor(n);
			if (nbr->containsPoint(p))
			{
				return nbr->getID();
			}
		}
		return NavMeshLocation::NO_NODE;
	}

	Resource* NavMeshLocalizer::load(const std::string& fileName)
	{
		NavMeshPtr mesh;

		mesh = loadNavMesh(fileName);

		NavMeshLocalizer* nml = new NavMeshLocalizer(fileName);
		nml->_navMesh = mesh;
		return nml;
	}

	NavMeshLocalizerPtr loadNavMeshLocalizer(const std::string& fileName, bool usePlanner)
	{
		Resource* rsrc = ResourceManager::getResource(fileName,
		                                              &NavMeshLocalizer::load,
		                                              NavMeshLocalizer::LABEL);
		if (rsrc == 0x0)
		{
			return NULL;
		}
		NavMeshLocalizer* nml = dynamic_cast<NavMeshLocalizer *>(rsrc);
		if (nml == 0x0)
		{
			return NULL;
		}

		if (usePlanner)
		{
			if (nml->getPlanner() == 0x0)
			{
				PathPlanner* planner = new PathPlanner(nml->getNavMesh());
				nml->setPlanner(planner);
			}
		}

		return NavMeshLocalizerPtr(nml);
	}
}
