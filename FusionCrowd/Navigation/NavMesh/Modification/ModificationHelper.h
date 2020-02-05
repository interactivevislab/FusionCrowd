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
		bool IsPointsOnLine(Vector2 v0, Vector2 v1, Vector2 v2, float delta = 1e-2f);
		Vector2 GetLineIntersectionPoint(Vector2& p0, Vector2& p1, Vector2& o1, Vector2& o2);

		void ConcaveHull(std::vector<Vector2>& poly);
		void ResetNodePolySequence(NavMeshNode& node);
		void RemoveRepeatedVertex(NavMeshNode& node);
		void SimplifyPoly(std::vector<Vector2> &poly);
		bool IsClockwise(std::vector<Vector2>& polygon);

		bool ValidateModificator(NodeModificator * modificator, std::vector<NodeModificator*>& modifications);
		std::vector<Vector2> FindPolyAndSegmentCrosspoints(Vector2 v0, Vector2 v1, NavMeshPoly* poly, bool ray_mode = false);

		bool right(Vector2& a, Vector2& b, Vector2& c);
		float area(Vector2& a, Vector2& b, Vector2& c);
		bool rightOn(Vector2& a, Vector2& b, Vector2& c);
		bool leftOn(Vector2& a, Vector2& b, Vector2& c);
	};
};

