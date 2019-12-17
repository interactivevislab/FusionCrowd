#pragma once
#include "NavMesh.h"
#include "../SpatialQuery/NavMeshSpatialQuery.h"
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
		bool side;
	};


	class NavMeshModifyer
	{
	public:
		NavMeshModifyer(NavMesh& navmesh, std::shared_ptr<NavMeshLocalizer> localizer, NavMeshSpatialQuery* spatial_query);
		~NavMeshModifyer();
		float CutPolygonFromMesh(FCArray<NavMeshVetrex> & polygon);

	private:
		//global data and methods
		NavMesh&  _navmesh;
		std::shared_ptr<NavMeshLocalizer> _localizer;
		NavMeshSpatialQuery* _spatial_query;

		std::vector<NodeModificator*> _modifications;

		float SplitPolyByNodes(FCArray<NavMeshVetrex> & polygon);
		void FinaliseNodes();
		int Finalize();

		std::vector<NavMeshNode*> _addednodes;
		std::vector<NavMeshEdge*> _addededges;
		std::vector<NavMeshObstacle*> _addedobstacles;
		std::vector<size_t> _nodes_ids_to_delete;
		std::vector<Vector2> _addedvertices;
		std::vector<Vector2> _global_polygon;
		size_t next_node_id = 0;

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
		void FillAddedVertices();
		Vector2 FindVortexCrossPoint(Vector2 v0, Vector2 v1, int& out_prev_cross_id);
		void CopyVortexObstacles(NavMeshNode* updnode, int j, Vector2 j0vert,
			Vector2 j1vert, Vector2 j2vert, bool node_side0, bool node_side1, bool onestrict = false);
		void CopyVortexEdges(NavMeshNode* updnode, int j, Vector2 j0vert,
			Vector2 j1vert, Vector2 j2vert, bool node_side0, bool node_side1, bool onestrict = false);
		std::vector<unsigned int> crosspoints_ids;
		std::vector<Vector2> crosspoints;
		std::vector<unsigned int> crosspoints_prev_vertex_ids;

		//utility
		bool IsPointUnderLine(Vector2 v0, Vector2 v1, Vector2 point, bool reverse = false, bool strict = false);
		bool IsClockwise(FCArray<NavMeshVetrex> & polygon);
		bool IsTriangleClockwise(Vector2 v0, Vector2 v1, Vector2 v2);
		unsigned int AddVertex(Vector2 v);
		std::vector<Vector2> FindPolyAndSegmentCrosspoints(Vector2 v0, Vector2 v1, NavMeshPoly* poly);
		bool IsSegmentsIntersects(Vector2 v00, Vector2 v01, Vector2 v10, Vector2 v11);
		NavMeshNode* GetNodeById(size_t id);
		bool ProcessEdge(NavMeshEdge* edge);
		size_t GetNextNodeID();
		void FixPoly(NavMeshNode& node);
		void FixConcavePoly();

		float tres = 0;
	};
}

