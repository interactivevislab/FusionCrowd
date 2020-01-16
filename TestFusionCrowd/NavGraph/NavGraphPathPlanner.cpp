#include "pch.h"

#include <map>
#include <set>
#include <queue>

#include "NavGraphPathPlanner.h"

namespace FusionCrowd
{
	NavGraphPathPlanner::NavGraphPathPlanner(std::shared_ptr<NavGraph> navGraph) : _navGraph(navGraph)
	{ }

	NavGraphRoute NavGraphPathPlanner::GetRoute(size_t nodeFrom, size_t nodeTo) const
	{
		std::map<size_t, float> distance;
		for(auto & node : _navGraph->GetAllNodes())
		{
			distance.insert({node.id, INFINITY});
		}

		distance.at(nodeFrom) = 0;

		std::set<size_t> visited;
		std::queue<size_t> remainingNodes;

		visited.insert(nodeFrom);

		for(auto & edge : _navGraph->GetOutEdges(nodeFrom))
			remainingNodes.push(edge.nodeTo);

		while(!remainingNodes.empty())
		{
			size_t nodeId = remainingNodes.front(); remainingNodes.pop();
			auto curDist = distance.at(nodeId);

			for(auto & edge : _navGraph->GetOutEdges(nodeId))
			{
				auto d = (_navGraph->GetNode(nodeId).position - _navGraph->GetNode(edge.nodeTo).position).Length();

				if(curDist + d < distance.at(edge.nodeTo))
				{
					distance.at(edge.nodeTo) = curDist + d;
				}

				if(visited.find(edge.nodeTo) != visited.end())
					remainingNodes.push(edge.nodeTo);
			}
		}

		size_t current = nodeTo;
		std::vector<size_t> route_nodes;
		route_nodes.push_back(current);

		float curDistance = distance.at(current);
		while(current != nodeFrom)
		{
			float minDist = curDistance;
			size_t next = -1;
			for(auto & e : _navGraph->GetInEdges(current))
			{
				if(distance.at(e.nodeFrom) < minDist)
				{
					minDist = distance.at(e.nodeFrom);
					next = e.nodeFrom;
				}
			}

			current = next;
			route_nodes.push_back(current);
		}

		return NavGraphRoute(route_nodes);
	}
}
