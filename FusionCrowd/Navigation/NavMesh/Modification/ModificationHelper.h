#pragma once

#include "../NavMesh.h"
#include <vector>
#include "Modificator.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	namespace ModificationHelper
	{
		bool IsPointUnderLine(Vector2 v0, Vector2 v1, Vector2 point, bool reverse = false, bool strict = false);
		bool IsClockwise(FCArray<NavMeshVetrex> & polygon);
		std::vector<Vector2> FindPolyAndSegmentCrosspoints(Vector2 v0, Vector2 v1, NavMeshPoly* poly, bool ray_mode = false);
		void ConcaveHull(std::vector<Vector2>& poly);
		void ResetNodePolySequence(NavMeshNode& node);
		void SimplifyPoly(std::vector<Vector2> &poly);
		bool ValidateModificator(NodeModificator * modificator, std::vector<NodeModificator*>& modifications);
		bool IsPointsOnLine(Vector2 v0, Vector2 v1, Vector2 v2, float delta = 1e-2f);
	};
};

