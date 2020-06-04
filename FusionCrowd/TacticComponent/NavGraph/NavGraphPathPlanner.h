#pragma once

#include <memory>

#include "Navigation/NavGraph/NavGraph.h"
#include "Math/Util.h"



namespace FusionCrowd
{
	class NavGraphRoute
	{
	public:
		NavGraphRoute()
		{ }

		NavGraphRoute(std::vector<DirectX::SimpleMath::Vector2> points) : points(points)
		{ }

	public:
		std::vector<DirectX::SimpleMath::Vector2> points;
	};

	class NavGraphPathPlanner
	{
	public:
		explicit NavGraphPathPlanner(std::shared_ptr<NavGraph> navGraph);

		NavGraphRoute GetRoute(DirectX::SimpleMath::Vector2 from, DirectX::SimpleMath::Vector2 to) const;

	private:
		std::vector<size_t> GetRouteNodes(size_t from, size_t to) const;

		std::shared_ptr<NavGraph> _navGraph;
	};
}
