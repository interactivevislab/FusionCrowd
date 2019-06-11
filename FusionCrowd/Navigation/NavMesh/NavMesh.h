#pragma once

#include <string>
#include <map>
#include <vector>

#include "Config.h"
#include "NavMeshEdge.h"
#include "NavMeshObstacle.h"
#include "NavMeshNode.h"
#include "Math/Util.h"

namespace FusionCrowd
{
	// forward declarations
	class NavMesh;
	class NavMeshNode;
	class NavMeshEdge;
	class NavMeshLocalizer;
	class PathPlanner;

	class NMNodeGroup
	{
	public:
		NMNodeGroup() : _first(0), _last(0)
		{
		}

		NMNodeGroup(size_t first, size_t last) : _first(first), _last(last)
		{
		}

		inline size_t getFirst() const { return _first; }
		inline size_t getLast() const { return _last; }
		inline size_t getGlobalId(unsigned int i) const { return _first + i; }
		inline size_t groupSize() const { return static_cast<size_t>(_last - _first + 1); }

	private:
		size_t _first;
		size_t _last;
	};

	class FUSION_CROWD_API NavMesh
	{
	public:
		NavMesh(const std::string& name);
		~NavMesh();
		//Load
		static std::shared_ptr<NavMesh> Load(const std::string& FileName);

		void clear();
		bool finalize();
		//Vertex
		void SetVertexCount(size_t count);
		void SetVertex(unsigned int i, float x, float y);
		inline DirectX::SimpleMath::Vector2* GetVertices() { return & vertices[0]; }
		//Edge
		void SetEdgeCount(size_t count);
		NavMeshEdge& GetEdge(unsigned int i);
		//Obstacle
		void SetObstacleCount(size_t count);
		NavMeshObstacle& GetObstacle(unsigned int i);
		int getObstacleCount() { return obstCount; }
		//Node
		bool AddGroup(const std::string& grpName, size_t grpSize);
		NavMeshNode& GetNode(unsigned int i);
		inline size_t getNodeCount() const { return nCount; }
		const NMNodeGroup* getNodeGroup(const std::string& grpName) const;

	protected:
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
}