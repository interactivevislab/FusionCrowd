#pragma once

#include <string>
#include <map>
#include <vector>

#include "../../Math/vector.h"
#include "NavMeshEdge.h"
#include "NavMeshObstacle.h"
#include "NavMeshNode.h"

// forward declarations
class NavMesh;
class NavMeshNode;
class NavMeshEdge;

class NMNodeGroup
{
public:
	NMNodeGroup() : _first(0), _last(0) {}
	NMNodeGroup(unsigned int first, unsigned int last) : _first(first), _last(last) {}
	inline size_t getGlobalId(unsigned int i) const { return _first + i; }
	inline size_t groupSize() const { return static_cast<size_t>(_last - _first + 1); }

	unsigned int _first;
	unsigned int _last;
};

class NavMesh
{
public:
	NavMesh();
	~NavMesh();
	//Load
	bool Load(const std::string FileName);
	void Destroy() { delete this; };
	bool finalize();
	//Vertex
	void SetVertexCount(size_t count);
	void SetVertex(unsigned int i, float x, float y);
	inline FusionCrowd::Math::Vector2 * GetVertices() { return & vertices[0]; }
	//Edge
	void SetEdgeCount(size_t count);
	NavMeshEdge & GetEdge(unsigned int i);
	//Obstacle
	void SetObstacleCount(size_t count);
	NavMeshObstacle & GetObstacle(unsigned int i);
	//Node
	bool AddGroup(const std::string & grpName, size_t grpSize);
	NavMeshNode & GetNode(unsigned int i);

private:
	std::string fileName;
	size_t vCount;
	FusionCrowd::Math::Vector2* vertices;
	int eCount;
	NavMeshEdge* edges;
	int obstCount;
	NavMeshObstacle* obstacles;
	int nCount;
	NavMeshNode* nodes;
	std::map<const std::string, NMNodeGroup> nodeGroups;
};

