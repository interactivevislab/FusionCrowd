#pragma once

#include "INavMeshPublic.h"
#include "Navigation/NavMesh/NavMesh.h"


namespace FusionCrowd
{
	FCArray<NavMeshVetrex> NavMeshHelper::LoadNavMeshVertices(const char * inNavMeshfileName)
	{
		auto navMesh = NavMesh::Load(inNavMeshfileName);
		FCArray<NavMeshVetrex> vertices(navMesh->GetVertexCount());
		navMesh->GetVertices(vertices);
		return std::move(vertices);
	}
}
