#pragma once

#include <string>
#include <map>
#include <vector>

#include "../../Config.h"
#include "NavMeshEdge.h"
#include "NavMeshObstacle.h"
#include "NavMeshNode.h"
#include "Resource.h"
#include "../../MathUtil.h"

// forward declarations
class NavMesh;
class NavMeshNode;
class NavMeshEdge;
class NavMeshLocalizer;
class PathPlanner;

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

class FUSION_CROWD_API NavMesh: public Resource
{
public:
	NavMesh(const std::string & name);
	~NavMesh();
	//Load
	static Resource * Load(const std::string & FileName);
	void Destroy() { delete this; };
	void clear();
	bool finalize();
	//Vertex
	void SetVertexCount(size_t count);
	void SetVertex(unsigned int i, float x, float y);
	inline DirectX::SimpleMath::Vector2 * GetVertices() { return & vertices[0]; }
	//Edge
	void SetEdgeCount(size_t count);
	NavMeshEdge & GetEdge(unsigned int i);
	//Obstacle
	void SetObstacleCount(size_t count);
	NavMeshObstacle & GetObstacle(unsigned int i);
	int getObstacleCount() { return obstCount; }
	//Node
	bool AddGroup(const std::string & grpName, size_t grpSize);
	NavMeshNode & GetNode(unsigned int i);
	inline size_t getNodeCount() const { return nCount; }
	const NMNodeGroup * getNodeGroup(const std::string & grpName) const;

	static constexpr const char* LABEL = "navmesh";

	virtual const std::string & getLabel() const { return LABEL; }
//protected:
	std::string fileName;
	size_t vCount;
	DirectX::SimpleMath::Vector2* vertices;
	int eCount;
	NavMeshEdge* edges;
	int obstCount;
	NavMeshObstacle* obstacles;
	int nCount;
	NavMeshNode* nodes;
	std::map<const std::string, NMNodeGroup> nodeGroups;

};

typedef ResourcePtr<NavMesh> NavMeshPtr;

NavMeshPtr loadNavMesh(const std::string & fileName);


