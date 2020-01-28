#pragma once
#include "Navigation/NavMesh/NavMesh.h"
#include "Navigation/SpatialQuery/NavMeshSpatialQuery.h"
#include "ModificationHelper.h"
#include "NavMeshModification.h"

#include <vector>

namespace FusionCrowd {
	enum ModificationTypes {
		SPLIT,
		CUT_POLY,
		CUT_CURVE
	};

	struct NodeModificator {
		NavMeshNode* node;
		std::vector<DirectX::SimpleMath::Vector2> polygon_to_cut;
		std::vector<unsigned int> polygon_vertex_ids;
		ModificationTypes modification_type;
		bool side;
		bool correct = true;
	};

	class ModificationProcessor
	{
	public:
		ModificationProcessor(NavMesh& navmesh, std::shared_ptr<NavMeshLocalizer> localizer, NavMeshSpatialQuery* spatial_query);
		~ModificationProcessor();
		float CutPolygonFromMesh(FCArray<NavMeshVetrex> & polygon);

	private:
		NavMeshModification _modification;

		std::vector<NodeModificator*> _node_modificators;

		float SplitPolyByNodes(FCArray<NavMeshVetrex> & polygon);

		//node change data and methods
		NavMeshPoly* _current_node_poly;
		NavMeshNode* _current_node;
		std::vector<DirectX::SimpleMath::Vector2> _local_polygon;
		std::vector<unsigned int> _local_polygon_vertex_ids;
		bool _side;

		int Initialize(NodeModificator * modificator);
		int CutPolyFromCurrentNode();
		int SplitNode();
		int CutCurveFromCurrentNode();

		//vortex crosspoints
		void FillAddedVertices(bool isCurve);
		bool ValidateModificator(NodeModificator * modificator);
		DirectX::SimpleMath::Vector2 FindVortexCrossPoint(DirectX::SimpleMath::Vector2 v0, DirectX::SimpleMath::Vector2 v1);
		void CopyVortexObstacles(NavMeshNode* updnode, int j, DirectX::SimpleMath::Vector2 j0vert,
			DirectX::SimpleMath::Vector2 j1vert, DirectX::SimpleMath::Vector2 j2vert, bool node_side0, bool node_side1, bool onestrict = false);
		void CopyVortexEdges(NavMeshNode* updnode, int j, DirectX::SimpleMath::Vector2 j0vert,
			DirectX::SimpleMath::Vector2 j1vert, DirectX::SimpleMath::Vector2 j2vert, bool node_side0, bool node_side1, bool onestrict = false);
		std::vector<unsigned int> crosspoints_ids;
		std::vector<DirectX::SimpleMath::Vector2> crosspoints;

		//debug variable
		float tres = 0;
	};
}

