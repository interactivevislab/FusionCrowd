#pragma once
#include "NavMesh.h"

using namespace DirectX::SimpleMath;
namespace FusionCrowd {
	class NavMeshModifyer
	{
	public:
		NavMeshModifyer(NavMesh& navmesh);
		~NavMeshModifyer();
		float CutPolygonFromMesh(FCArray<NavMeshVetrex> & polygon);

	private:
		NavMesh&  _navmesh;

		int polygonvertices_count = 0;
		Vector2* polygonvertices;

		int addednodes_count = 0;
		NavMeshNode* addednodes;

		int addedvertices_count = 0;
		Vector2* addedvertices;

		int crosspoints_prev_vertex_ids_num = 0;
		int* crosspoints_prev_vertex_ids;

		NavMeshPoly* _current_poly;

		int CutPolyFromCurrentNode();
		int SplitNode(Vector2 v0, Vector2 v1);
		int CutCurveFromCurrentNode();

		int Initialize(FCArray<NavMeshVetrex> & polygon);
		void FillAddedVertices();
		int Finalize();
		bool IsPointUnderLine(Vector2 v0, Vector2 v1, Vector2 point, bool reverse = false, bool strict = false);
		Vector2 FindCrossPoint(Vector2 v0, Vector2 v1, int& out_prev_cross_id);
		bool IsClockwise(FCArray<NavMeshVetrex> & polygon);
	};
}

