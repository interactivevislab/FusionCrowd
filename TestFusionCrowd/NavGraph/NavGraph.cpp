#include "pch.h"


#include "NavGraph.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	NavGraph NavGraph::LoadFromStream(std::istream & istream)
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
			float x, y;
			if(! (istream >> x >> y))
			{
				throw "Invalid format";
			}

			nodes.push_back({n, Vector2(x, y)});
		}

		size_t edgeCount;
		if(! (istream >> edgeCount))
		{
			throw "Invalid format";
		}

		for(size_t e = 0; e < edgeCount; e++)
		{
			size_t nFrom, nTo;
			float width, weight;
			bool bidirectional;

			if(! (istream >> nFrom >> nTo >> width >> weight >> bidirectional))
			{
				throw "Invalid format";
			}

			edges.push_back({ e, nFrom, nTo, weight, width});
			if(bidirectional)
				edges.push_back({ edgeCount + e, nTo, nFrom, weight, width});
		}

		return NavGraph(nodes, edges);
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

	std::vector<NavGraphEdge> NavGraph::GetOutEdges(size_t fromNodeId) const
	{
		return _outEdges.at(fromNodeId);
	}

	std::vector<NavGraphEdge> NavGraph::GetInEdges(size_t toNodeId) const
	{
		return _inEdges.at(toNodeId);
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
