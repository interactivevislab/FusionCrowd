#pragma once

#include "../Math/vector.h"
#include "Route.h"
#include "../NavComponents/NavMesh/NavMesh.h"
#include "../Goal/Goal.h"
#include "../Agent.h"
#include "../Config.h"

// Forward Declaration
class NavMeshLocalizer;
class PathPlanner;


class FUSION_CROWD_API PortalPath
{
public:
	PortalPath(const FusionCrowd::Math::Vector2 & startPos, const Goal * goal,
		const PortalRoute * route, float agentRadius);
	~PortalPath();
	void setPreferredDirection(const FusionCrowd::Agent * agent, float headingCos,
		Agents::PrefVelocity & pVel);
	unsigned int updateLocation(const FusionCrowd::Agent * agent, const NavMeshPtr & navMesh,
		const NavMeshLocalizer * localizer, PathPlanner * planner);
	unsigned int getNode() const;
	inline size_t getWayPointCount() const { return _route->getPortalCount(); }
	FusionCrowd::Math::Vector2 getWayPoint(size_t i) const;
	inline const Goal * getGoal() const { return _goal; }
	inline FusionCrowd::Math::Vector2 getGoalCentroid() const { return _goal->getCentroid(); }
	inline unsigned int getEndNode() const { return _route->getEndNode(); }
	unsigned int getNode(size_t i) const;
	inline size_t getCurrentPortal() const { return _currPortal; }
	inline size_t getPortalCount() const { return _route->getPortalCount(); }
	inline const WayPortal * getPortal(size_t i) const { return _route->getPortal(i); }
	void setWaypoints(size_t start, size_t end, const FusionCrowd::Math::Vector2 & p0,
		const FusionCrowd::Math::Vector2 & dir);
protected:
	const PortalRoute *	_route;
	const Goal * _goal;
	size_t	_currPortal;
	void computeCrossing(const FusionCrowd::Math::Vector2 & startPos, float agentRadius);
	FusionCrowd::Math::Vector2*	_waypoints;
	FusionCrowd::Math::Vector2*	_headings;
	void replan(const FusionCrowd::Math::Vector2 & startPos, unsigned int startNode, unsigned int endNode,
		float minWidth, PathPlanner * planner);
};

