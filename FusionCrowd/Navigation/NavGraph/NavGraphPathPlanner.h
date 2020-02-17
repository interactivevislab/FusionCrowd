#pragma once

#include <memory>

#include "Navigation/NavGraph/NavGraph.h"



namespace FusionCrowd
{
	class NavGraphRoute
	{
	public:
		NavGraphRoute(std::vector<size_t> nodes) : nodes(nodes)
		{ }

	public:
		std::vector<size_t> nodes;
	};

	class NavGraphPathPlanner
	{
	public:
		explicit NavGraphPathPlanner(std::shared_ptr<NavGraph> navGraph);

		std::shared_ptr <NavGraphRoute> GetRoute(size_t nodeFrom, size_t nodeTo) const;

	private:
		std::shared_ptr<NavGraph> _navGraph;
	};
}
