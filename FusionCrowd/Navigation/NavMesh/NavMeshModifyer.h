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
		float DivideNode(NavMeshNode* node, DirectX::SimpleMath::Vector2* v0, DirectX::SimpleMath::Vector2* v1);
	};
}

