#include "NavGraph.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	std::shared_ptr <NavGraph> NavGraph::LoadFromStream(std::istream & istream)
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
			size_t id;
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
			size_t id;
			size_t nFrom, nTo;
			float width, weight;

			if(! (istream >> id >> nFrom >> nTo >> width >> weight))
			{
				throw "Invalid format";
			}

			edges.push_back({ id, nFrom, nTo, weight, width});
		}

		return std::make_shared<NavGraph>(nodes, edges);
	}


	NavGraph::NavGraph(std::vector<NavGraphNode> nodes, std::vector<NavGraphEdge> edges)
	{
		for(auto & n : nodes)
		{
			_nodes[n.id] = n;
		}

		for(auto & e : edges)
		{
			_outEdges[e.nodeFrom].push_back(e);
			_inEdges[e.nodeTo].push_back(e);
		}
	}

	const NavGraphNode& NavGraph::GetNode(size_t id) const
	{
		return _nodes.at(id);
	}

	const size_t NavGraph::GetClosestNodeIdByPosition(Vector2 p, std::unordered_set<NavGraphNode> nodes)
	{
		float minDistance = INFINITY;
		size_t retID = 0;
		for (auto node : nodes)
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

	std::vector<NavGraphEdge> NavGraph::GetOutEdges(size_t fromNodeId) const
	{
		auto e = _outEdges.find(fromNodeId);

		if (e == _outEdges.end())
			return std::vector<NavGraphEdge>();

		return e->second;
	}

	std::vector<NavGraphEdge> NavGraph::GetInEdges(size_t toNodeId) const
	{
		auto e = _inEdges.find(toNodeId);

		if (e == _inEdges.end())
			return std::vector<NavGraphEdge>();

		return e->second;
	}

	std::unordered_set<NavGraphNode> NavGraph::GetOutNeighbours(size_t fromNodeId) const
	{
		if(_outEdges.find(fromNodeId) == _outEdges.end())
			return std::unordered_set<NavGraphNode>();

		std::unordered_set<NavGraphNode> result;

		for(auto e : _outEdges.at(fromNodeId))
		{
			result.insert(_nodes.at(e.nodeTo));
		}

		return result;
	}

	std::unordered_set<NavGraphNode> NavGraph::GetInNeighbours(size_t toNodeId) const
	{
		if(_inEdges.find(toNodeId) == _inEdges.end())
			return std::unordered_set<NavGraphNode>();

		std::unordered_set<NavGraphNode> result;

		for(auto e : _inEdges.at(toNodeId))
		{
			result.insert(_nodes.at(e.nodeFrom));
		}

		return result;
	}

	std::unordered_set<NavGraphNode> NavGraph::GetAllNodes() const
	{
		std::unordered_set<NavGraphNode> result;

		for(auto & n : _nodes)
		{
			result.insert(n.second);
		}

		return result;
	}
}
