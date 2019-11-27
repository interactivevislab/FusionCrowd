#pragma once
#include "NavMesh.h"
#include <vector>

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
	};


	class NavMeshModifyer
	{
	public:
		NavMeshModifyer(NavMesh& navmesh, std::shared_ptr<NavMeshLocalizer> localizer);
		~NavMeshModifyer();
		float CutPolygonFromMesh(FCArray<NavMeshVetrex> & polygon);

	private:
		//global data and methods
		NavMesh&  _navmesh;
		std::shared_ptr<NavMeshLocalizer> _localizer;

		std::vector<NodeModificator*> _modifications;

		float SplitPolyByNodes(FCArray<NavMeshVetrex> & polygon);

		std::vector<NavMeshNode*> _addednodes;
		std::vector<unsigned int> _nodes_ids_to_delete;
		std::vector<Vector2> _addedvertices;
		std::vector<Vector2> _global_polygon;

		int Finalize();

		//node change data and methods
		NavMeshPoly* _current_node_poly;
		std::vector<Vector2> _local_polygon;
		std::vector<unsigned int> _local_polygon_vertex_ids;

		int CutPolyFromCurrentNode();
		int SplitNode();
		int CutCurveFromCurrentNode();

		int Initialize(NodeModificator * modificator);

		//vortex crosspoints
		void FillAddedVertices();
		Vector2 FindVortexCrossPoint(Vector2 v0, Vector2 v1, int& out_prev_cross_id);
		std::vector<unsigned int> crosspoints_ids;
		std::vector<unsigned int> crosspoints_prev_vertex_ids;

		//utility
		bool IsPointUnderLine(Vector2 v0, Vector2 v1, Vector2 point, bool reverse = false, bool strict = false);
		bool IsClockwise(FCArray<NavMeshVetrex> & polygon);
		bool IsTriangleClockwise(Vector2 v0, Vector2 v1, Vector2 v2);
		unsigned int AddVertex(Vector2 v);
		std::vector<Vector2> FindPolyAndSegmentCrosspoints(Vector2 v0, Vector2 v1, NavMeshPoly* poly);
		bool IsSegmentsIntersects(Vector2 v00, Vector2 v01, Vector2 v10, Vector2 v11);
		NavMeshNode* GetNodeById(unsigned int id);
	};
}

