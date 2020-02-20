#pragma once

#include "../NavMesh.h"
#include "Navigation/SpatialQuery/NavMeshSpatialQuery.h"
#include <vector>

class ModificationProcessor;

namespace FusionCrowd {
	class NavMeshModification
	{
	public:
		NavMeshModification(NavMesh& navmesh, std::shared_ptr<NavMeshLocalizer> localizer, NavMeshSpatialQuery* spatial_query);
		~NavMeshModification();

	private:
		//input data
		const float min_width = 1e-2f;
		NavMesh&  _navmesh;
		std::shared_ptr<NavMeshLocalizer> _localizer;
		NavMeshSpatialQuery* _spatial_query;

		//modification
		std::vector<NavMeshNode*> _addednodes;
		std::vector<NavMeshEdge*> _addededges;
		std::vector<NavMeshObstacle*> _addedobstacles;
		std::vector<size_t> _nodes_ids_to_delete;
		std::vector<DirectX::SimpleMath::Vector2> _addedvertices;
		std::vector<DirectX::SimpleMath::Vector2> _global_polygon;
		size_t next_node_id = 0;

		//Finalize
		int Finalize();
		/*Some times unnecessary obstacles spawns wich have same points as edges. This function deletes such obstalces*/
		void RemoveObstaclesOnEdges();
		/*Check is both obstacles point on node poly*/
		bool ValidateObstalce(NavMeshObstacle* obst);
		void FinalizeVertices();
		void FinalizeNodes();
		void FinalizeEdges();
		bool ProcessEdge(NavMeshEdge* edge);
		void FillNodeEdgeObstaclesArrays();
		void RemoveNodesInsidePoly(std::vector<DirectX::SimpleMath::Vector2> poly);
		void Clear();

		//Interface
		unsigned int AddVertex(DirectX::SimpleMath::Vector2 v);
		NavMeshNode* FindNode(size_t id);
		unsigned int GetNextNodeID();
		NavMeshNode* GetGlobalPolygonNodeByVertexId(size_t id);
		friend ModificationProcessor;
	};
}

