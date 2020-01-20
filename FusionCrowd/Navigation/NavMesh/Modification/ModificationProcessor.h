#pragma once
#include "../NavMesh.h"
#include "Navigation/SpatialQuery/NavMeshSpatialQuery.h"
#include <vector>
#include "ModificationHelper.h"
#include "NavMeshModification.h"

using namespace DirectX::SimpleMath;
namespace FusionCrowd {
	enum ModificationTypes {
		SPLIT,
		CUT_POLY,
		CUT_CURVE
	};

	struct NodeModificator {
		NavMeshNode* node;
		std::vector<Vector2> polygon_to_cut;
		std::vector<unsigned int> polygon_vertex_ids;
		ModificationTypes modification_type;
		bool side;
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
		std::vector<Vector2> _local_polygon;
		std::vector<unsigned int> _local_polygon_vertex_ids;
		bool _side;

		int Initialize(NodeModificator * modificator);
		int CutPolyFromCurrentNode();
		int SplitNode();
		int CutCurveFromCurrentNode();

		//vortex crosspoints
		void FillAddedVertices(bool isCurve);
		Vector2 FindVortexCrossPoint(Vector2 v0, Vector2 v1, bool& success);
		void CopyVortexObstacles(NavMeshNode* updnode, int j, Vector2 j0vert,
			Vector2 j1vert, Vector2 j2vert, bool node_side0, bool node_side1, bool onestrict = false);
		void CopyVortexEdges(NavMeshNode* updnode, int j, Vector2 j0vert,
			Vector2 j1vert, Vector2 j2vert, bool node_side0, bool node_side1, bool onestrict = false);
		std::vector<unsigned int> crosspoints_ids;
		std::vector<Vector2> crosspoints;

		//debug variable
		float tres = 0;
	};
}

