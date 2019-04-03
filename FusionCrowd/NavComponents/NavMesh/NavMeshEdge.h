#pragma once

#include <fstream>

#include "../../Config.h"
#include "../../Math/vector.h"
#include "../../Path/PrefVelocity.h"
#include "NavMeshNode.h"

// Forward declarations
class NavMesh;
class NavMeshNode;

class FUSION_CROWD_API NavMeshEdge
{
public:
	NavMeshEdge();
	~NavMeshEdge();

	inline FusionCrowd::Math::Vector2 getP0() const { return _point; }
	inline FusionCrowd::Math::Vector2 getP0(float dist) const { return _point + _dir * dist; }
	inline FusionCrowd::Math::Vector2 getP1() const { return _point + _dir * _width; }
	inline FusionCrowd::Math::Vector2 getP1(float dist) const {
		return _point + _dir * (_width - dist);
	}
	inline FusionCrowd::Math::Vector2 getDirection() const { return _dir; }
	NavMeshNode * getFirstNode() const { return _node0; }
	NavMeshNode * getOtherByID(unsigned int id) const;
	NavMeshNode * getOtherByPtr(const NavMeshNode * node);
	const NavMeshNode * getOtherByPtr(const NavMeshNode * node) const;

	inline void setPoint(const FusionCrowd::Math::Vector2 & p) { _point.set(p); }
	inline void setDirection(const FusionCrowd::Math::Vector2 & d) { _dir.set(d); }
	inline void setWidth(float w) { _width = w; }
	inline float getWidth() const { return _width; }
	inline void setNodes(NavMeshNode * n0, NavMeshNode * n1) { _node0 = n0; _node1 = n1; }
	inline FusionCrowd::Math::Vector2 getPoint(float t) const { return _point + t * _dir; }
	bool pointClear(const FusionCrowd::Math::Vector2 & pos, float radius, float param) const;
	FusionCrowd::Math::Vector2 targetPoint(const FusionCrowd::Math::Vector2 & pos, float radius) const;
	FusionCrowd::Math::Vector2	getClearDirection(const FusionCrowd::Math::Vector2 & pos, float radius,
		const FusionCrowd::Math::Vector2 & dir) const;
	void setClearDirections(const FusionCrowd::Math::Vector2 & pos, float radius,
		const FusionCrowd::Math::Vector2 & dir, Agents::PrefVelocity & pVel) const;
	float getSqDist(const FusionCrowd::Math::Vector2 & pt) const;
	float getSqDist(const FusionCrowd::Math::Vector2 & pt, FusionCrowd::Math::Vector2 & nearPt) const;
	float getDist(const FusionCrowd::Math::Vector2 & pt) const { return FusionCrowd::Math::sqr(getSqDist(pt)); }
	float getNodeDistance(float minWidth);
	inline float getNodeDistance() const { return _distance; }
	bool loadFromAscii(std::ifstream & f, FusionCrowd::Math::Vector2 * vertices);
	bool pointOnLeft(unsigned int id) const;
	bool pointOnLeft(const NavMeshNode * node) const;
	friend class NavMesh;
protected:
	FusionCrowd::Math::Vector2	_point;
	FusionCrowd::Math::Vector2 _dir;
	float	_width;
	float	_distance;
	NavMeshNode *	_node0;
	NavMeshNode *	_node1;
};

