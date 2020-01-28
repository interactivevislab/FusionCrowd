#pragma once

#include "Navigation/NavMesh/NavMesh.h"
#include "Math/Util.h"
#include "Export/INavMeshPublic.h"
#include "Modificator.h"
#include <vector>

namespace FusionCrowd
{
	namespace ModificationHelper
	{
		bool IsPointUnderLine(DirectX::SimpleMath::Vector2 v0, DirectX::SimpleMath::Vector2 v1, DirectX::SimpleMath::Vector2 point, bool reverse = false, bool strict = false);
		bool IsClockwise(FCArray<NavMeshVetrex> & polygon);
		std::vector<DirectX::SimpleMath::Vector2> FindPolyAndSegmentCrosspoints(DirectX::SimpleMath::Vector2 v0, DirectX::SimpleMath::Vector2 v1, NavMeshPoly* poly, bool ray_mode = false);
		void ConcaveHull(std::vector<DirectX::SimpleMath::Vector2>& poly);
		void ResetNodePolySequence(NavMeshNode& node);
		void SimplifyPoly(std::vector<DirectX::SimpleMath::Vector2> &poly);
		bool ValidateModificator(NodeModificator * modificator, std::vector<NodeModificator*>& modifications);
		bool IsPointsOnLine(DirectX::SimpleMath::Vector2 v0, DirectX::SimpleMath::Vector2 v1, DirectX::SimpleMath::Vector2 v2, float delta = 1e-2f);
	};
};

