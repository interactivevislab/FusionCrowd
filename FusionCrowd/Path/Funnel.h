#pragma once

#include "../Math/vector.h"
#include "../Math/consts.h"

#include <list>

// FORWARD DECLARATION
class PortalPath;
class FunnelPlanner;

class FunnelApex
{
public:
	FunnelApex(size_t id, const FusionCrowd::Math::Vector2 & point) : _id(id), _pos(point) {}
	inline void set(size_t id, const FusionCrowd::Math::Vector2 & point) { _id = id; _pos.set(point); }

	friend class FunnelPlanner;
protected:
	size_t	_id;
	FusionCrowd::Math::Vector2 _pos;
};

class FunnelEdge
{
public:
	FunnelEdge() {}
#ifdef SIMPLE_FUNNEL
	FunnelEdge(size_t id, const Vector2 & dir) :_id(id), _dir(dir) {}
#else
	FunnelEdge(size_t id, size_t end, const FusionCrowd::Math::Vector2 & dir, const FusionCrowd::Math::Vector2 & origin) :
		_id(id), _endID(end), _dir(dir), _origin(origin) {}
#endif
	inline bool isOnLeft(const FusionCrowd::Math::Vector2 & dir) const { return det(_dir, dir) > FusionCrowd::EPS; }
	inline bool isOnRight(const FusionCrowd::Math::Vector2 & dir) const { return det(dir, _dir) > FusionCrowd::EPS; }
	inline void set(size_t id, const FusionCrowd::Math::Vector2 & dir) { _id = id; _dir.set(dir); }
	friend class FunnelPlanner;
protected:
#ifdef SIMPLE_FUNNEL
	size_t	_id;
#else
	size_t	_id;
	size_t  _endID;
	FusionCrowd::Math::Vector2 _origin;
#endif
	FusionCrowd::Math::Vector2 _dir;
};

class FunnelPlanner
{
public:
	FunnelPlanner();
	~FunnelPlanner();
	void computeCrossing(float radius, const FusionCrowd::Math::Vector2 & startPos, PortalPath * path,
		size_t startPortal = 0);
#ifndef SIMPLE_FUNNEL
protected:
	std::list< FunnelEdge >	_left;
	std::list< FunnelEdge >	_right;
#endif
};

