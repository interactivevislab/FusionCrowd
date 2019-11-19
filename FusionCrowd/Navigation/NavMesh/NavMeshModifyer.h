#pragma once
#include "NavMesh.h"

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
		DirectX::SimpleMath::Vector2* polygonvertices;

		int addednodes_count = 0;
		NavMeshNode* addednodes;

		int addedvertices_count = 0;
		DirectX::SimpleMath::Vector2* addedvertices;

		int crosspoints_prev_vertex_ids_num = 0;
		int* crosspoints_prev_vertex_ids;

		NavMeshPoly* _current_poly;
		bool _leave_up_node;

		void Initialize(FCArray<NavMeshVetrex> & polygon);
		void FillAddedVertices();
		void ModifyHomeNode();
		void Finalize();
		DirectX::SimpleMath::Vector2 FindCrossPoint(DirectX::SimpleMath::Vector2 v0, DirectX::SimpleMath::Vector2 v1, int& out_prev_cross_id);
		float DivideNode(NavMeshNode* node, DirectX::SimpleMath::Vector2* v0, DirectX::SimpleMath::Vector2* v1);
	};
}

