#pragma once


#include "NavMeshEdge.h"
#include "NavMeshObstacle.h"
#include "NavMeshPoly.h"
#include "../../Math/vector.h"

// Forward declarations
class NavMesh;
class NavMeshEdge;
class PathPlanner;
class NavMeshObstacle;

class NavMeshNode
{
public:
	NavMeshNode();
	~NavMeshNode();

	NavMeshNode & operator=(const NavMeshNode & n);
	void setID(unsigned int id) { _id = id; }
	unsigned int getID() const { return _id; }
	inline void setCenter(const FusionCrowd::Math::Vector2 & c) { _center.set(c); }
	FusionCrowd::Math::Vector2 getCenter() const { return _center; }
	FusionCrowd::Math::Vector3 getCenter3D() const {
		return FusionCrowd::Math::Vector3(_center.x(), _poly.getElevation(_center), _center.y());
	}
	size_t getVertexCount() const { return _poly.vertCount; }
	void setVertices(const FusionCrowd::Math::Vector2 * vertices) { _poly.vertices = vertices; }
	unsigned int getVertexID(size_t i) const { return _poly.vertIDs[i]; }
	size_t getObstacleCount() const { return _obstCount; }
	const NavMeshObstacle * getObstacle(size_t i) const { return _obstacles[i]; }
	NavMeshObstacle * getObstacle(size_t i) { return _obstacles[i]; }
	size_t getNeighborCount() const { return _edgeCount; }
	const NavMeshNode * getNeighbor(size_t i) const;
	size_t getEdgeCount() const { return _edgeCount; }
	NavMeshEdge * getEdge(size_t i) { return _edges[i]; }
	const NavMeshEdge * getEdge(size_t i) const { return _edges[i]; }
	NavMeshEdge * getConnection(unsigned nodeID);
	bool containsPoint(const FusionCrowd::Math::Vector2 & point) const
	{
		return _poly.containsPoint(point);
	}
	bool loadFromAscii(std::ifstream & f);
	inline float getElevation(const FusionCrowd::Math::Vector2 & p) const {
		return _poly.getElevation(p);
	}
	inline FusionCrowd::Math::Vector2 getGradient() const { return _poly.getGradient(); }

	friend class NavMesh;
	friend class NavMeshEdge;
protected:
	NavMeshEdge **	_edges;
	size_t	_edgeCount;
	NavMeshObstacle ** _obstacles;
	size_t _obstCount;
	FusionCrowd::Math::Vector2		_center;
	NavMeshPoly		_poly;
	unsigned int	_id;
};

