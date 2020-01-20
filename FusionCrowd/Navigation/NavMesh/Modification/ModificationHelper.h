#pragma once

#include "../NavMesh.h"
#include <vector>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	namespace ModificationHelper
	{
		bool IsPointUnderLine(Vector2 v0, Vector2 v1, Vector2 point, bool reverse = false, bool strict = false);
		bool IsClockwise(FCArray<NavMeshVetrex> & polygon);
		std::vector<Vector2> FindPolyAndSegmentCrosspoints(Vector2 v0, Vector2 v1, NavMeshPoly* poly);
		bool IsSegmentsIntersects(Vector2 v00, Vector2 v01, Vector2 v10, Vector2 v11);
		void ConcaveHull(std::vector<Vector2>& poly);
		void ResetNodePolySequence(NavMeshNode& node);
		void SimplifyPoly(std::vector<Vector2> &poly);
	};
};

