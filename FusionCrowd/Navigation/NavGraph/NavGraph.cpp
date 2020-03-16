#include "NavGraph.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	std::unique_ptr <NavGraph> NavGraph::LoadFromStream(std::istream & istream)
	{
		std::vector<NavGraphNode> nodes;
		std::vector<NavGraphEdge> edges;

		size_t nodeCount = 0;
		if(! (istream >> nodeCount))
		{
			throw "Invalid format";
		}
		for(size_t n = 0; n < nodeCount; n++)
		{
			NavGraphNodeId id;
			float x, y;
			if(! (istream >> id >> x >> y))
			{
				throw "Invalid format";
			}

			nodes.push_back({id, Vector2(x, y)});
		}

		size_t edgeCount;
		if(! (istream >> edgeCount))
		{
			throw "Invalid format";
		}

		for(size_t e = 0; e < edgeCount; e++)
		{
			NavGraphEdgeId id;
			NavGraphNodeId nFrom, nTo;
			float width, weight;

			if(! (istream >> id >> nFrom >> nTo >> width >> weight))
			{
				throw "Invalid format";
			}

			edges.push_back({ id, nFrom, nTo, weight, width});
		}

		return std::make_unique<NavGraph>(nodes, edges);
	}

	NavGraph::NavGraph(std::vector<NavGraphNode> nodes, std::vector<NavGraphEdge> edges)
	{
		for(const auto & n : nodes)
		{
			_nodes[n.id] = n;
		}

		for(const auto & e : edges)
		{
			_outEdges[e.nodeFrom].push_back(e);
			_inEdges[e.nodeTo].push_back(e);
		}
	}

	const NavGraphNode& NavGraph::GetNode(NavGraphNodeId id) const
	{
		return _nodes.at(id);
	}

	const NavGraphNodeId NavGraph::GetClosestNodeIdByPosition(Vector2 p, std::unordered_set<NavGraphNode> nodes)
	{
		float minDistance = INFINITY;
		NavGraphNodeId retID = 0;
		for (const auto & node : nodes)
		{
			Vector2 nodePos = node.position;

			float dist = p.Distance(p, nodePos);
			if (minDistance > dist)
			{
				minDistance = dist;
				retID = node.id;
			}
		}

		return retID;
	}

	const DirectX::SimpleMath::Vector2 NavGraph::GetClosiestPointAndNodeId(DirectX::SimpleMath::Vector2 p, NavGraphNodeId& nodeId) {
		float min_dist = INFINITY;
		Vector2 res;
		nodeId = -1;

		for (auto & node : _nodes)
		{
			float dist2 = (node.second.position - p).LengthSquared();
			if(dist2 < min_dist)
			{
				min_dist = dist2;
				res = p;
				nodeId = node.second.id;
			}
		}

		if(nodeId != -1)
		{
			return res;
		}

		for(auto & edgeList : _outEdges)
		{
			auto & node = _nodes[edgeList.first];

			for(auto & e : edgeList.second)
			{
				Vector2 AB = _nodes[e.nodeTo].position - node.position;
				Vector2 AP = p - node.position;
				float sqrAB = AB.LengthSquared();
				float t = (AP.x * AB.x + AP.y*AB.y) / sqrAB;
				t = t < 0.0f ? 0.0f : t;
				t = t > 1.0f ? 1.0f : t;
				Vector2 tmp_res = node.position + t * AB;
				float dist = Vector2::DistanceSquared(tmp_res, p);
				if (dist < min_dist) {
					res = tmp_res;
					min_dist = dist;
					nodeId = e.nodeTo;
				}
			}
		}

		return res;
	}

	std::vector<NavGraphEdge> NavGraph::GetOutEdges(NavGraphNodeId from) const
	{
		auto e = _outEdges.find(from);

		if (e == _outEdges.end())
			return std::vector<NavGraphEdge>();

		return e->second;
	}

	std::vector<NavGraphEdge> NavGraph::GetInEdges(NavGraphNodeId to) const
	{
		auto e = _inEdges.find(to);

		if (e == _inEdges.end())
			return std::vector<NavGraphEdge>();

		return e->second;
	}

	std::unordered_set<NavGraphNode> NavGraph::GetOutNeighbours(NavGraphNodeId from) const
	{
		if(_outEdges.find(from) == _outEdges.end())
		{
			return std::unordered_set<NavGraphNode>();
		}

		std::unordered_set<NavGraphNode> result;

		for(const auto & e : _outEdges.at(from))
		{
			result.insert(_nodes.at(e.nodeTo));
		}

		return result;
	}

	std::unordered_set<NavGraphNode> NavGraph::GetInNeighbours(NavGraphNodeId to) const
	{
		if(_inEdges.find(to) == _inEdges.end())
		{
			return std::unordered_set<NavGraphNode>();
		}

		std::unordered_set<NavGraphNode> result;

		for(const auto & e : _inEdges.at(to))
		{
			result.insert(_nodes.at(e.nodeFrom));
		}

		return result;
	}

	std::unordered_set<NavGraphNode> NavGraph::GetAllNodes() const
	{
		std::unordered_set<NavGraphNode> result;

		for(const auto & n : _nodes)
		{
			result.insert(n.second);
		}

		return result;
	}
}
