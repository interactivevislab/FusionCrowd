#pragma once

#include <string>
#include <map>
#include <vector>

#include "Export/FCArray.h"
#include "Export/Export.h"
#include "Export/INavMeshPublic.h"
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
	class ModificationProcessor;
	class NavMeshModification;
	class EdgeObstacleReplaner;

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

	class NavMesh : public INavMeshPublic
	{
	public:
		NavMesh();
		~NavMesh();
		//Load
		static std::shared_ptr<NavMesh> Load(std::istream& f);

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
		NavMeshNode& GetNodeByPos(unsigned int i);
		NavMeshNode* GetNodeByID(unsigned int id);
		inline size_t getNodeCount() const { return nCount; }
		const NMNodeGroup* getNodeGroup(const std::string& grpName) const;

		size_t GetVersion() const;
		void IncVersion();

	public:
		// INavMeshPublicAPI
		size_t GetVertexCount();
		bool GetVertices(FCArray<NavMeshVetrex> & output);
		size_t GetNodesCount();
		size_t GetNodeVertexCount(size_t node_id);
		bool GetNodeVertexInfo(FCArray<int> & output, size_t node_id);
		size_t GetEdgesCount();
		bool GetEdges(FCArray<EdgeInfo> & output);
		size_t GetObstaclesCount();
		bool GetObstacles(FCArray<EdgeInfo> & output);
		bool ExportNavMeshToFile(char* file_name) override;

	protected:
		void CheckObstaclesDirection();
		void ReverseCycle(size_t obstId);

		std::string fileName;
		size_t vCount;
		DirectX::SimpleMath::Vector2* vertices;
		int eCount;
		NavMeshEdge* edges;
		int obstCount;
		std::vector<NavMeshObstacle> obstacles;
		int nCount;
		NavMeshNode* nodes;
		std::map<const std::string, NMNodeGroup> nodeGroups;

		friend ModificationProcessor;
		friend NavMeshModification;
		friend EdgeObstacleReplaner;

	private:
		size_t _version = 0;
	};
}
