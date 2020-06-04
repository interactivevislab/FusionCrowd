#include "NavMeshLocalizer.h"

#include "NavMeshNode.h"
#include "TacticComponent/NavMesh/Path/PathPlanner.h"
#include "TacticComponent/NavMesh/Path/PortalPath.h"

#include <limits>
#include <iostream>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	void NavMeshLocation::setNode(unsigned int nodeID)
	{
		if (_hasPath)
		{
			_path = nullptr;

		}
		_hasPath = false;
		_nodeID = nodeID;
	}

	void NavMeshLocation::clearPath()
	{
		if (_hasPath)
		{
			unsigned int node = _path->getNode();
			_path = nullptr;
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

	void NavMeshLocation::setPath(std::shared_ptr<PortalPath> path)
	{
		_path = path;
		_hasPath = true;
		_nodeID = std::numeric_limits<size_t>::max();
	}

	std::shared_ptr<PortalPath> NavMeshLocation::getPath()
	{
		if(_hasPath)
			return _path;
		else
			return nullptr;
	}

	NavMeshLocalizer::NavMeshLocalizer(const std::string& name, bool usePlanner) : _navMesh(0x0), _trackAll(false), _planner(0x0)
	{
		std::ifstream f;
		f.open(name, std::ios::in);

		if (f.is_open())
		{
			_navMesh = NavMesh::Load(f);
		} else
		{
			throw std::ios_base::failure("Can't load navmesh");
		}

		const size_t NODE_COUNT = _navMesh->getNodeCount();

		if (usePlanner)
		{
			std::shared_ptr<PathPlanner> planner = std::make_shared<PathPlanner>(_navMesh);
			setPlanner(planner);
		}

		std::vector<QuadTree::Box> nodeBoxes;
		for(size_t nodeId = 0; nodeId < NODE_COUNT; nodeId++)
		{
			nodeBoxes.push_back({_navMesh->GetNodeByPos(nodeId).GetBB(), nodeId });
		}

		_nodeBBTree = std::make_unique<QuadTree>(nodeBoxes);
	}

	NavMeshLocalizer::~NavMeshLocalizer()
	{
	}

	unsigned int NavMeshLocalizer::getNodeId(const Vector2& p) const
	{
		return findNodeBlind(p, 0.f);
	}

	std::vector<size_t> NavMeshLocalizer::findNodesCrossingBB(BoundingBox bb) {
		return _nodeBBTree->GetIntersectingBBIds(bb);
	}

	unsigned int NavMeshLocalizer::findNodeBlind(const Vector2& p, float tgtElev) const
	{
		float elevDiff = 1e6f;
		unsigned int maxNode = NavMeshLocation::NO_NODE;

		for(size_t nodeId : _nodeBBTree->GetContainingBBIds(p))
		//for (size_t nodeId = 0; nodeId <_navMesh->getNodeCount(); nodeId++)
		{
			const NavMeshNode* node = _navMesh->GetNodeByID(nodeId);
			if (node->deleted) continue;
			if (node->containsPoint(p))
			{
				float hDiff = fabs(node->getElevation(p) - tgtElev);
				if (hDiff < elevDiff)
				{
					maxNode = nodeId;
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
			node = findNodeInRange(p, grp->getFirst(), grp->getLast() + 1);

			if (node == NavMeshLocation::NO_NODE && searchAll)
			{
				node = findNodeInRange(p, 0, grp->getFirst());
				if (node == NavMeshLocation::NO_NODE)
				{
					// TODO(curds01) 10/1/2016 - This cast is bad because I can lose precision
					// (after I get 4 billion nodes...)
					const unsigned int TOTAL_NODES = static_cast<unsigned int>(_navMesh->getNodeCount());
					node = findNodeInRange(p, grp->getFirst() + 1, TOTAL_NODES);
				}
			}
		}
		return node;
	}

	unsigned int NavMeshLocalizer::findNodeInRange(const Vector2& p, unsigned int start, unsigned int stop) const
	{
		for (unsigned int n = start; n < stop; ++n)
		{
			const NavMeshNode& node = _navMesh->GetNodeByPos(n);
			if (node.containsPoint(p))
			{
				return n;
			}
		}
		return NavMeshLocation::NO_NODE;
	}

	unsigned int NavMeshLocalizer::testNeighbors(const NavMeshNode& node, const Vector2& p) const
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


	DirectX::SimpleMath::Vector2 NavMeshLocalizer::GetClosestAvailablePoint(DirectX::SimpleMath::Vector2 p) {
		if (findNodeBlind(p) != NavMeshLocation::NO_NODE)
		{
			return p;
		}

		float min_dist = INFINITY;
		Vector2 res;
		auto* vertices = _navMesh->GetVertices();
		for (int i = _navMesh->getNodeCount() - 1; i >= 0; i--)
		{
			const auto & node = _navMesh->GetNodeByPos(i);
			if (node.deleted)
				continue;

			const size_t vCount = node.getVertexCount();
			for (size_t v = 0; v < vCount; v++)
			{
				Vector2 vertex1 = vertices[node.getVertexID(v)];
				Vector2 vertex2 = vertices[(node.getVertexID(v) + 1) % vCount];

				auto projection = Math::projectOnSegment(vertex1, vertex2, p);

				const float d = Vector2::DistanceSquared(p, projection);
				if (d < min_dist)
				{
					min_dist = d;
					res = projection;
				}
			}
		}
		if (min_dist == INFINITY)
		{
			throw 1;
		}

		return res;
	}

	void NavMeshLocalizer::Update(std::vector<NavMeshNode*>& added_nodes, std::vector<size_t>& del_nodes) {
		std::vector<QuadTree::Box> added_boxes = std::vector<QuadTree::Box>();
		for (int i = 0; i < added_nodes.size(); i++) {
			added_boxes.push_back({ added_nodes[i]->GetBB(), added_nodes[i]->_id });
		}
		_nodeBBTree->UpdateTree(added_boxes, del_nodes);
	}
}
