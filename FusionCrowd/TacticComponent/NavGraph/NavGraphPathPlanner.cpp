#include <map>
#include <set>
#include <queue>

#include "Navigation/NavGraph/NavGraph.h"

#include "NavGraphPathPlanner.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	NavGraphPathPlanner::NavGraphPathPlanner(std::shared_ptr<NavGraph> navGraph) : _navGraph(navGraph)
	{ }

	std::vector<NavGraphNodeId> NavGraphPathPlanner::GetRouteNodes(NavGraphNodeId nodeFrom, NavGraphNodeId nodeTo) const
	{
		if (nodeFrom == nodeTo)
		{
			return { nodeFrom };
		}

		std::vector<NavGraphNodeId> route_nodes;

		std::map<NavGraphNodeId, float> distance;
		for (const auto& node : _navGraph->GetAllNodes())
		{
			distance.insert({ node.id, INFINITY });
		}
		distance.at(nodeFrom) = 0;

		auto distComparator = [&](NavGraphNodeId left, NavGraphNodeId right)
		{
			// we need node with smallest distance to be at the top of the queue, hence "greater than"
			return distance[left] > distance[right];
		};

		std::priority_queue<NavGraphNodeId, std::vector<NavGraphNodeId>, decltype(distComparator)> remainingNodes(distComparator);
		std::map<NavGraphNodeId, NavGraphNodeId> prev;
		std::set<NavGraphNodeId> unvisited;
		for(const auto & node : _navGraph->GetAllNodes())
		{
			unvisited.insert(node.id);
		}

		unvisited.erase(nodeFrom);
		remainingNodes.push(nodeFrom);
		prev[nodeFrom] = nodeFrom;

		bool thereIsRoute = false;
		while (!remainingNodes.empty())
		{
			NavGraphNodeId nodeId = remainingNodes.top(); remainingNodes.pop();
			float curDist = distance.at(nodeId);
			unvisited.erase(nodeId);

			for (const auto& edge : _navGraph->GetOutEdges(nodeId))
			{
				auto d = (_navGraph->GetNode(nodeId).position - _navGraph->GetNode(edge.nodeTo).position).Length();

				if (curDist + d < distance[edge.nodeTo])
				{
					distance[edge.nodeTo] = curDist + d;
					prev[edge.nodeTo] = nodeId;
				}

				if (unvisited.find(edge.nodeTo) != unvisited.end())
				{
					remainingNodes.push(edge.nodeTo);
				}
			}
		}

		if (isinf(distance[nodeTo]))
		{
			return { };
		}

		NavGraphNodeId current = nodeTo;
		do
		{
			route_nodes.push_back(current);
			current = prev[current];
		} while(current != nodeFrom);

		return route_nodes;
	}

	NavGraphRoute NavGraphPathPlanner::GetRoute(DirectX::SimpleMath::Vector2 from, DirectX::SimpleMath::Vector2 to) const
	{
		NavGraphNodeId nodeFrom;
		Vector2 init_point = _navGraph->GetClosiestPointAndNodeId(from, nodeFrom);

		NavGraphNodeId nodeTo;
		Vector2 goal_point = _navGraph->GetClosiestPointAndNodeId(to, nodeTo);

		std::vector<NavGraphNodeId> routeNodesReversed = GetRouteNodes(nodeFrom, nodeTo);

		if(routeNodesReversed.size() == 0)
		{
			return NavGraphRoute({ from });
		}

		if(routeNodesReversed.size() == 1)
		{
			return NavGraphRoute({ from, to });
		}

		std::vector<Vector2> path;

		path.push_back(from);
		path.push_back(init_point);

		auto nodeId = routeNodesReversed.rbegin();
		while(nodeId != routeNodesReversed.rend())
		{
			path.push_back(_navGraph->GetNode(*nodeId).position);
			nodeId++;
		}

		path.push_back(goal_point);
		path.push_back(to);

		return NavGraphRoute(path);
	}
}