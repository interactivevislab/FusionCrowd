#pragma once

#include "Route.h"
#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/NavMesh/NavMesh.h"
#include "StrategyComponent/Goal/Goal.h"
#include "Agent.h"
#include "Export/Config.h"
#include "Math/Util.h"

#include <memory>

namespace FusionCrowd
{
	class NavMeshLocalizer;
	class PathPlanner;

	class PortalPath
	{
	public:
		PortalPath(const DirectX::SimpleMath::Vector2 & startPos, const Goal & goal, const PortalRoute* route, float agentRadius);
		~PortalPath();
		void setPrefVelocity(AgentSpatialInfo & agent, float headingCos, float timeStep);
		unsigned int updateLocation(const AgentSpatialInfo & agent, const std::shared_ptr<NavMesh> navMesh,
		                            const std::shared_ptr<NavMeshLocalizer> localizer, const std::shared_ptr<PathPlanner> planner);
		unsigned int getNode() const;
		inline size_t getWayPointCount() const { return _route->getPortalCount(); }
		DirectX::SimpleMath::Vector2 getWayPoint(size_t i) const;
		inline const Goal & getGoal() const { return _goal; }
		inline DirectX::SimpleMath::Vector2 getGoalCentroid() const { return _goal.getCentroid(); }
		inline unsigned int getEndNode() const { return _route->getEndNode(); }
		unsigned int getNode(size_t i) const;
		inline size_t getCurrentPortal() const { return _currPortal; }
		inline size_t getPortalCount() const { return _route->getPortalCount(); }
		inline const WayPortal* getPortal(size_t i) const { return _route->getPortal(i); }
		void setWaypoints(size_t start, size_t end, const DirectX::SimpleMath::Vector2& p0,
		                  const DirectX::SimpleMath::Vector2& dir);

		bool IsValid(size_t navMeshVersion) const;

		std::vector<DirectX::SimpleMath::Vector2> _headings;
	protected:
		const PortalRoute* _route;
		const Goal _goal;
		size_t _currPortal;

		void computeCrossing(const DirectX::SimpleMath::Vector2& startPos, float agentRadius);
		std::vector<DirectX::SimpleMath::Vector2> _waypoints;
		void replan(const DirectX::SimpleMath::Vector2& startPos, unsigned int startNode, unsigned int endNode,
		            float minWidth, const std::shared_ptr<PathPlanner> planner);
	};
}
