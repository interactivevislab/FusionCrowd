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
		bool IsPointUnderLine(DirectX::SimpleMath::Vector2 v0, DirectX::SimpleMath::Vector2 v1,
			DirectX::SimpleMath::Vector2 point, bool reverse = false, bool strict = false);
		bool IsPointsOnLine(DirectX::SimpleMath::Vector2 v0, DirectX::SimpleMath::Vector2 v1, DirectX::SimpleMath::Vector2 v2, float delta = 1e-2f);
		DirectX::SimpleMath::Vector2 GetLineIntersectionPoint(DirectX::SimpleMath::Vector2& p0, DirectX::SimpleMath::Vector2& p1,
			DirectX::SimpleMath::Vector2& o1, DirectX::SimpleMath::Vector2& o2);

		void ConcaveHull(std::vector<DirectX::SimpleMath::Vector2>& poly);
		void ResetNodePolySequence(NavMeshNode& node);
		void RemoveDuplicateVerticesFromNodePoly(NavMeshNode& node);
		void SimplifyPoly(std::vector<DirectX::SimpleMath::Vector2> &poly, bool makeConcave = true);
		bool IsClockwise(std::vector<DirectX::SimpleMath::Vector2>& polygon);

		bool ValidateModificator(NodeModificator * modificator, std::vector<NodeModificator*>& modifications);
		std::vector<DirectX::SimpleMath::Vector2> FindPolyAndSegmentCrosspoints(DirectX::SimpleMath::Vector2 v0,
			DirectX::SimpleMath::Vector2 v1, NavMeshPoly* poly, bool ray_mode = false);

		bool right(DirectX::SimpleMath::Vector2& a, DirectX::SimpleMath::Vector2& b, DirectX::SimpleMath::Vector2& c);
		float area(DirectX::SimpleMath::Vector2& a, DirectX::SimpleMath::Vector2& b, DirectX::SimpleMath::Vector2& c);
		bool rightOn(DirectX::SimpleMath::Vector2& a, DirectX::SimpleMath::Vector2& b, DirectX::SimpleMath::Vector2& c);
		bool leftOn(DirectX::SimpleMath::Vector2& a, DirectX::SimpleMath::Vector2& b, DirectX::SimpleMath::Vector2& c);
	};
};

