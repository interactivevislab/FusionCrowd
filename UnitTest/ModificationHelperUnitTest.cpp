#include "stdafx.h"
#include "CppUnitTest.h"

#include "Navigation/NavMesh/Modification/ModificationHelper.h"


using namespace Microsoft::VisualStudio::CppUnitTestFramework;

using namespace FusionCrowd;
using namespace DirectX::SimpleMath;

namespace UnitTest
{
	TEST_CLASS(ModificationHelperUnitTest)
	{
	public:
#pragma region ConcaveHull
		TEST_METHOD(ConcaveHull__DontModifyConcavePoly)
		{
			std::vector<Vector2> poly = {
				Vector2(0, 0),
				Vector2(0, 1),
				Vector2(1, 1),
				Vector2(1, 0)
			};

			ModificationHelper::ConcaveHull(poly);

			Assert::IsTrue(std::find(poly.begin(), poly.end(), Vector2(0, 0)) != poly.end(), L"Vertex missing");
			Assert::IsTrue(std::find(poly.begin(), poly.end(), Vector2(0, 1)) != poly.end(), L"Vertex missing");
			Assert::IsTrue(std::find(poly.begin(), poly.end(), Vector2(1, 1)) != poly.end(), L"Vertex missing");
			Assert::IsTrue(std::find(poly.begin(), poly.end(), Vector2(1, 0)) != poly.end(), L"Vertex missing");
		}

		TEST_METHOD(ConcaveHull__SimpleCase)
		{
			std::vector<Vector2> poly = {
				Vector2(0, 0),
				Vector2(0, 1),
				Vector2(1, 0),
				Vector2(0, -1),
				Vector2(-1, 0)
			};

			ModificationHelper::ConcaveHull(poly);

			Assert::IsFalse(std::find(poly.begin(), poly.end(), Vector2(0, 0)) != poly.end(), L"Vertex was not deleted");

			Assert::IsTrue(std::find(poly.begin(), poly.end(), Vector2(0, 1)) != poly.end(),  L"Vertex missing");
			Assert::IsTrue(std::find(poly.begin(), poly.end(), Vector2(0, -1)) != poly.end(), L"Vertex missing");
			Assert::IsTrue(std::find(poly.begin(), poly.end(), Vector2(1, 0)) != poly.end(),  L"Vertex missing");
			Assert::IsTrue(std::find(poly.begin(), poly.end(), Vector2(-1, 0)) != poly.end(), L"Vertex missing");
		}

		TEST_METHOD(ConcaveHull__StarCase)
		{
			std::vector<Vector2> poly = {
				Vector2( 0,  5),
				Vector2( 1,  1),
				Vector2( 5,  0),
				Vector2(-1,  1),
				Vector2( 0, -5),
				Vector2(-1, -1),
				Vector2(-5,  0),
				Vector2( 1, -1),
			};

			ModificationHelper::ConcaveHull(poly);

			Assert::IsTrue(poly.size() == 4, L"Wrong number of vertices");

			Assert::IsFalse(std::find(poly.begin(), poly.end(), Vector2( 1,  1)) != poly.end(), L"Vertex was not deleted");
			Assert::IsFalse(std::find(poly.begin(), poly.end(), Vector2(-1,  1)) != poly.end(), L"Vertex was not deleted");
			Assert::IsFalse(std::find(poly.begin(), poly.end(), Vector2( 1, -1)) != poly.end(), L"Vertex was not deleted");
			Assert::IsFalse(std::find(poly.begin(), poly.end(), Vector2(-1, -1)) != poly.end(), L"Vertex was not deleted");

			Assert::IsTrue(std::find(poly.begin(), poly.end(), Vector2(0,  5)) != poly.end(),  L"Vertex missing");
			Assert::IsTrue(std::find(poly.begin(), poly.end(), Vector2(0, -5)) != poly.end(),  L"Vertex missing");
			Assert::IsTrue(std::find(poly.begin(), poly.end(), Vector2(5,  0)) != poly.end(),  L"Vertex missing");
			Assert::IsTrue(std::find(poly.begin(), poly.end(), Vector2(-5, 0)) != poly.end(),  L"Vertex missing");
		}

		TEST_METHOD(ConcaveHull__RemoveTwoConsequent)
		{
			std::vector<Vector2> poly = {
				Vector2( 1,  1),
				Vector2( 0, .5),
				Vector2( 0,  0),
				Vector2(-1,  1),
				Vector2(-1, -1),
				Vector2( 1, -1),
			};

			ModificationHelper::ConcaveHull(poly);

			Assert::IsTrue(poly.size() == 4, L"Wrong number of vertices");

			Assert::IsFalse(std::find(poly.begin(), poly.end(), Vector2(0,  0)) != poly.end(), L"Vertex was not deleted");
			Assert::IsFalse(std::find(poly.begin(), poly.end(), Vector2(0, .5)) != poly.end(), L"Vertex was not deleted");

			Assert::IsTrue(std::find(poly.begin(), poly.end(), Vector2( 1,  1)) != poly.end(), L"Vertex missing");
			Assert::IsTrue(std::find(poly.begin(), poly.end(), Vector2(-1,  1)) != poly.end(), L"Vertex missing");
			Assert::IsTrue(std::find(poly.begin(), poly.end(), Vector2( 1, -1)) != poly.end(), L"Vertex missing");
			Assert::IsTrue(std::find(poly.begin(), poly.end(), Vector2(-1, -1)) != poly.end(), L"Vertex missing");
		}
#pragma endregion ConcaveHull

#pragma region PointsOnLine
		TEST_METHOD(IsPointsOnLine__Yes)
		{
			Vector2 p1(-1, 0);
			Vector2 p2(0, 0);
			Vector2 p3(2, 0);

			Assert::IsTrue(ModificationHelper::IsPointsOnLine(p1, p2, p3), L"Points must be on line");
		}

		TEST_METHOD(IsPointsOnLine__Yes2)
		{
			Vector2 p1(0, -1);
			Vector2 p2(0, 0);
			Vector2 p3(0, 2);

			Assert::IsTrue(ModificationHelper::IsPointsOnLine(p1, p2, p3), L"Points must be on line");
		}

		TEST_METHOD(IsPointsOnLine__No)
		{
			Vector2 p1(-1, 0);
			Vector2 p2(0, 1);
			Vector2 p3(2, 0);

			Assert::IsFalse(ModificationHelper::IsPointsOnLine(p1, p2, p3), L"Points must not be on line");
		}

		TEST_METHOD(IsPointsOnLine__No2)
		{
			Vector2 p1(0, -1);
			Vector2 p2(1, 1);
			Vector2 p3(0, 2);

			Assert::IsFalse(ModificationHelper::IsPointsOnLine(p1, p2, p3), L"Points must not be on line");
		}

		TEST_METHOD(IsPointsOnLine__Precision)
		{
			Vector2 p1(0, 0);
			Vector2 p2(1, 0);
			Vector2 p3(2, .002);

			Assert::IsTrue(ModificationHelper::IsPointsOnLine(p1, p2, p3, 0.01f),   L"Points must be on line");

			Assert::IsFalse(ModificationHelper::IsPointsOnLine(p1, p2, p3, 0.001f), L"Points must not be on line");
		}
#pragma endregion PointsOnLine

#pragma region GetLineIntersectionPoint
		TEST_METHOD(GetLineIntersectionPoint__InTheMiddle)
		{
			Vector2 p1(-1, -1);
			Vector2 p2( 1,  1);
			Vector2 p3( 1, -1);
			Vector2 p4(-1,  1);

			Assert::IsTrue(ModificationHelper::GetLineIntersectionPoint(p1, p2, p3, p4) == Vector2(0, 0),  L"Intersection is in the wrong place");
		}

		TEST_METHOD(GetLineIntersectionPoint__AtTheEnd)
		{
			Vector2 p1(-1, 0);
			Vector2 p2( 1, 0);
			Vector2 p3( 0, 0);
			Vector2 p4( 0, 10);

			Assert::IsTrue(ModificationHelper::GetLineIntersectionPoint(p1, p2, p3, p4) == Vector2(0, 0),  L"Intersection is in the wrong place");
		}

		TEST_METHOD(GetLineIntersectionPoint__AtTheEnd2)
		{
			Vector2 p1(-1, 0);
			Vector2 p2( 1, 0);
			Vector2 p3( 0, -10);
			Vector2 p4( 0, 0);

			Assert::IsTrue(ModificationHelper::GetLineIntersectionPoint(p1, p2, p3, p4) == Vector2(0, 0),  L"Intersection is in the wrong place");
		}

		TEST_METHOD(GetLineIntersectionPoint__AtTheEnd3)
		{
			Vector2 p1(-1, 0);
			Vector2 p2( 1, 0);
			Vector2 p3( 1, 0);
			Vector2 p4( 1, 1);

			Assert::IsTrue(ModificationHelper::GetLineIntersectionPoint(p1, p2, p3, p4) == Vector2(1, 0),  L"Intersection is in the wrong place");
		}
#pragma endregion

#pragma region RemoveRepeatedVertex
		TEST_METHOD(RemoveRepeatedVertex__NoRepeatedVertices)
		{
			NavMeshNode node;

			Vector2* vertices = new Vector2[4]
			{
				Vector2(-1, -1),
				Vector2( 1,  1),
				Vector2( 1, -1),
				Vector2(-1,  1)
			};

			unsigned int * vIDs = new unsigned int[4] { 0, 1, 2, 3 };
			node._poly.initialize(4, vIDs);
			node._poly.SetVertices(vertices);

			ModificationHelper::RemoveDuplicateVerticesFromNodePoly(node);
			auto & poly = node.getPoly();

			Assert::IsTrue(poly.vertCount == 4, L"Wrong number of vertices");

			auto end = poly.vertices + 4;
			Assert::IsTrue(std::find(poly.vertices, end, Vector2(-1, -1)) != end, L"Missing vertex");
			Assert::IsTrue(std::find(poly.vertices, end, Vector2( 1,  1)) != end, L"Missing vertex");
			Assert::IsTrue(std::find(poly.vertices, end, Vector2( 1, -1)) != end, L"Missing vertex");
			Assert::IsTrue(std::find(poly.vertices, end, Vector2(-1,  1)) != end, L"Missing vertex");
		}

		TEST_METHOD(RemoveRepeatedVertex__OneRepetition)
		{
			NavMeshNode node;
			Vector2* vertices = new Vector2[5]
			{
				Vector2(-1, -1),
				Vector2( 1,  1),

				Vector2( 1, -1),
				Vector2( 1, -1),

				Vector2(-1,  1)
			};
			unsigned int * vIDs = new unsigned int[5] { 0, 1, 2, 3, 4 };
			node._poly.initialize(5, vIDs);
			node._poly.SetVertices(vertices);

			ModificationHelper::RemoveDuplicateVerticesFromNodePoly(node);
			auto & poly = node.getPoly();

			Assert::IsTrue(poly.vertCount == 4, L"Wrong number of vertices");
			Assert::IsTrue(poly.getVertexByPos(0) == Vector2(-1, -1), L"Missing vertex");
			Assert::IsTrue(poly.getVertexByPos(1) == Vector2( 1,  1), L"Missing vertex");
			Assert::IsTrue(poly.getVertexByPos(2) == Vector2( 1, -1), L"Missing vertex");
			Assert::IsTrue(poly.getVertexByPos(3) == Vector2(-1,  1), L"Missing vertex");
		}

		TEST_METHOD(RemoveRepeatedVertex__FourCopiesOfTheSameVertex)
		{
			NavMeshNode node;
			Vector2* vertices = new Vector2[7]
			{
				Vector2(-1, -1),
				Vector2( 1,  1),

				Vector2( 1, -1),
				Vector2( 1, -1),
				Vector2( 1, -1),
				Vector2( 1, -1),

				Vector2(-1,  1)
			};
			unsigned int * vIDs = new unsigned int[7] { 0, 1, 2, 3, 4, 5, 6 };
			node._poly.initialize(7, vIDs);
			node._poly.SetVertices(vertices);

			ModificationHelper::RemoveDuplicateVerticesFromNodePoly(node);
			auto & poly = node.getPoly();

			Assert::IsTrue(poly.vertCount == 4, L"Wrong number of vertices");
			Assert::IsTrue(poly.getVertexByPos(0) == Vector2(-1, -1), L"Missing vertex");
			Assert::IsTrue(poly.getVertexByPos(1) == Vector2( 1,  1), L"Missing vertex");
			Assert::IsTrue(poly.getVertexByPos(2) == Vector2( 1, -1), L"Missing vertex");
			Assert::IsTrue(poly.getVertexByPos(3) == Vector2(-1,  1), L"Missing vertex");
		}

		TEST_METHOD(RemoveRepeatedVertex__TwoCopiesOfTwoVertices)
		{
			NavMeshNode node;
			Vector2* vertices = new Vector2[6]
			{
				Vector2(-1, -1),
				Vector2( 1,  1),

				Vector2( 1, -1),
				Vector2( 1, -1),

				Vector2(-1,  1),
				Vector2(-1,  1)
			};
			unsigned int * vIDs = new unsigned int[6] { 0, 1, 2, 3, 4, 5 };
			node._poly.initialize(5, vIDs);
			node._poly.SetVertices(vertices);

			ModificationHelper::RemoveDuplicateVerticesFromNodePoly(node);
			auto & poly = node.getPoly();

			Assert::IsTrue(poly.vertCount == 4, L"Wrong number of vertices");
			Assert::IsTrue(poly.getVertexByPos(0) == Vector2(-1, -1), L"Missing vertex");
			Assert::IsTrue(poly.getVertexByPos(1) == Vector2( 1,  1), L"Missing vertex");
			Assert::IsTrue(poly.getVertexByPos(2) == Vector2( 1, -1), L"Missing vertex");
			Assert::IsTrue(poly.getVertexByPos(3) == Vector2(-1,  1), L"Missing vertex");
		}

		TEST_METHOD(RemoveRepeatedVertex__ThreeCopiesOfTwoVertices)
		{
			NavMeshNode node;
			Vector2* vertices = new Vector2[8]
			{
				Vector2(-1, -1),
				Vector2( 1,  1),

				Vector2( 1, -1),
				Vector2( 1, -1),
				Vector2( 1, -1),

				Vector2(-1,  1),
				Vector2(-1,  1),
				Vector2(-1,  1)
			};
			unsigned int * vIDs = new unsigned int[8] { 0, 1, 2, 3, 4, 5, 6, 7 };
			node._poly.initialize(8, vIDs);
			node._poly.SetVertices(vertices);

			ModificationHelper::RemoveDuplicateVerticesFromNodePoly(node);
			auto & poly = node.getPoly();

			Assert::IsTrue(poly.vertCount == 4, L"Wrong number of vertices");
			Assert::IsTrue(poly.getVertexByPos(0) == Vector2(-1, -1), L"Missing vertex");
			Assert::IsTrue(poly.getVertexByPos(1) == Vector2( 1,  1), L"Missing vertex");
			Assert::IsTrue(poly.getVertexByPos(2) == Vector2( 1, -1), L"Missing vertex");
			Assert::IsTrue(poly.getVertexByPos(3) == Vector2(-1,  1), L"Missing vertex");
		}

		TEST_METHOD(RemoveRepeatedVertex__AtTheEnds)
		{
			NavMeshNode node;
			Vector2* vertices = new Vector2[5]
			{
				Vector2(-1, -1),//Same vertex
				Vector2( 1,  1),
				Vector2( 1, -1),
				Vector2(-1,  1),
				Vector2(-1, -1) //Same vertex
			};
			unsigned int * vIDs = new unsigned int[5] { 0, 1, 2, 3, 4 };
			node._poly.initialize(5, vIDs);
			node._poly.SetVertices(vertices);

			ModificationHelper::RemoveDuplicateVerticesFromNodePoly(node);
			auto & poly = node.getPoly();

			Assert::IsTrue(poly.vertCount == 4, L"Wrong number of vertices");

			auto end = poly.vertIDs + 4;

			bool firstVertexIsKept = std::find(poly.vertIDs, end, 0) != end;
			bool lastVertexIsKept  = std::find(poly.vertIDs, end, 4) != end;

			Assert::IsFalse(firstVertexIsKept && lastVertexIsKept, L"Wrong vertex was removed");
		}
#pragma endregion

#pragma region IsClockwise
		TEST_METHOD(IsClockWise__Clockwise)
		{
			std::vector<Vector2> poly = {
				Vector2(1, 1),
				Vector2(1, 0),
				Vector2(0, 0),
				Vector2(0, 1),
			};

			Assert::IsTrue(ModificationHelper::IsClockwise(poly), L"Poly is clockwise");
		}

		TEST_METHOD(IsClockWise__CounterClockwise)
		{
			std::vector<Vector2> poly = {
				Vector2(0, 0),
				Vector2(1, 0),
				Vector2(1, 1),
				Vector2(0, 1),
			};

			Assert::IsFalse(ModificationHelper::IsClockwise(poly), L"Poly is counter clockwise");
		}
#pragma endregion

#pragma region FindPolyAndSegmentCrosspoints
		TEST_METHOD(FindPolyAndSegmentCrosspoints__ZeroCrosspoints)
		{
			auto vertices = new Vector2[4] {
				Vector2(1, 1),
				Vector2(1, 0),
				Vector2(0, 0),
				Vector2(0, 1),
			};
			auto vIDs = new unsigned int[4] { 0, 1, 2, 3 };

			NavMeshPoly poly;
			poly.initialize(4, vIDs);
			poly.vertices = vertices;

			Vector2 v0(5, 5), v1(2, 5);

			auto result = ModificationHelper::FindPolyAndSegmentCrosspoints(v0, v1, &poly);

			Assert::IsTrue(result.size() == 0,           L"Must be exactly zero cross points");
		}

		TEST_METHOD(FindPolyAndSegmentCrosspoints__OneEdgeCrosspoint)
		{
			auto vertices = new Vector2[4] {
				Vector2(1, 1),
				Vector2(1, 0),
				Vector2(0, 0),
				Vector2(0, 1),
			};
			auto vIDs = new unsigned int[4] { 0, 1, 2, 3 };

			NavMeshPoly poly;
			poly.initialize(4, vIDs);
			poly.vertices = vertices;

			Vector2 v0(.5, .5), v1(2, .5);

			auto result = ModificationHelper::FindPolyAndSegmentCrosspoints(v0, v1, &poly);

			Assert::IsTrue(result.size() == 1,           L"Must be exactly one cross point");
			Assert::IsTrue(result[0] == Vector2(1, 0.5), L"Wrong cross point");
		}

		TEST_METHOD(FindPolyAndSegmentCrosspoints__TwoEdgeCrosspoints)
		{
			auto vertices = new Vector2[4] {
				Vector2(1, 1),
				Vector2(1, 0),
				Vector2(0, 0),
				Vector2(0, 1),
			};
			auto vIDs = new unsigned int[4] { 0, 1, 2, 3 };

			NavMeshPoly poly;
			poly.initialize(4, vIDs);
			poly.vertices = vertices;

			Vector2 v0(-1, .5), v1(2, .5);

			auto result = ModificationHelper::FindPolyAndSegmentCrosspoints(v0, v1, &poly);

			Assert::IsTrue(result.size() == 2, L"Must be exactly two cross points");

			Assert::IsTrue(std::find(result.begin(), result.end(), Vector2(1, 0.5)) != result.end(), L"Missing cross point");
			Assert::IsTrue(std::find(result.begin(), result.end(), Vector2(0, 0.5)) != result.end(), L"Missing cross point");
		}

		TEST_METHOD(FindPolyAndSegmentCrosspoints__OneCornerCrosspoint)
		{
			auto vertices = new Vector2[4] {
				Vector2(1, 1),
				Vector2(1, 0),
				Vector2(0, 0),
				Vector2(0, 1),
			};
			auto vIDs = new unsigned int[4] { 0, 1, 2, 3 };

			NavMeshPoly poly;
			poly.initialize(4, vIDs);
			poly.vertices = vertices;

			Vector2 v0(.5, .5), v1(1.5, 1.5);

			auto result = ModificationHelper::FindPolyAndSegmentCrosspoints(v0, v1, &poly);

			Assert::IsTrue(result.size() == 1,         L"Must be exactly one cross point");
			Assert::IsTrue(result[0] == Vector2(1, 1), L"Wrong cross point");
		}

		TEST_METHOD(FindPolyAndSegmentCrosspoints__TwoCornerCrosspoints)
		{
			auto vertices = new Vector2[4] {
				Vector2(1, 1),
				Vector2(1, 0),
				Vector2(0, 0),
				Vector2(0, 1),
			};
			auto vIDs = new unsigned int[4] { 0, 1, 2, 3 };

			NavMeshPoly poly;
			poly.initialize(4, vIDs);
			poly.vertices = vertices;

			Vector2 v0(-.5, -.5), v1(2.5, 2.5);

			auto result = ModificationHelper::FindPolyAndSegmentCrosspoints(v0, v1, &poly);

			Assert::IsTrue(result.size() == 2, L"Must be exactly two cross points");

			Assert::IsTrue(std::find(result.begin(), result.end(), Vector2(1, 1)) != result.end(), L"Missing cross point");
			Assert::IsTrue(std::find(result.begin(), result.end(), Vector2(0, 0)) != result.end(), L"Missing cross point");
		}

		TEST_METHOD(FindPolyAndSegmentCrosspoints__SegmentOnTheSameLineAsEdgeAndCovering)
		{
			auto vertices = new Vector2[4] {
				Vector2(1, 1),
				Vector2(1, 0),
				Vector2(0, 0),
				Vector2(0, 1),
			};
			auto vIDs = new unsigned int[4] { 0, 1, 2, 3 };

			NavMeshPoly poly;
			poly.initialize(4, vIDs);
			poly.vertices = vertices;

			Vector2 v0(-1, 0), v1(5, 0);

			auto result = ModificationHelper::FindPolyAndSegmentCrosspoints(v0, v1, &poly);

			Assert::IsTrue(result.size() == 2, L"Must be exactly two cross points");

			Assert::IsTrue(std::find(result.begin(), result.end(), Vector2(1, 0)) != result.end(), L"Missing cross point");
			Assert::IsTrue(std::find(result.begin(), result.end(), Vector2(0, 0)) != result.end(), L"Missing cross point");
		}

		TEST_METHOD(FindPolyAndSegmentCrosspoints__SegmentOnTheSameLineAsEdgeAndNotCovering)
		{
			auto vertices = new Vector2[4] {
				Vector2(1, 1),
				Vector2(1, 0),
				Vector2(0, 0),
				Vector2(0, 1),
			};
			auto vIDs = new unsigned int[4] { 0, 1, 2, 3 };

			NavMeshPoly poly;
			poly.initialize(4, vIDs);
			poly.vertices = vertices;

			Vector2 v0(-1, 0), v1(.5, 0);

			auto result = ModificationHelper::FindPolyAndSegmentCrosspoints(v0, v1, &poly);

			Assert::IsTrue(result.size() == 2, L"Must be exactly two cross points");

			Assert::IsTrue(std::find(result.begin(), result.end(), Vector2(.5, 0)) != result.end(), L"Missing cross point");
			Assert::IsTrue(std::find(result.begin(), result.end(), Vector2(0, 0)) != result.end(),  L"Missing cross point");
		}

		TEST_METHOD(FindPolyAndSegmentCrosspoints__ZeroCrosspoints_RayMode)
		{
			auto vertices = new Vector2[4] {
				Vector2(1, 1),
				Vector2(1, 0),
				Vector2(0, 0),
				Vector2(0, 1),
			};
			auto vIDs = new unsigned int[4] { 0, 1, 2, 3 };

			NavMeshPoly poly;
			poly.initialize(4, vIDs);
			poly.vertices = vertices;

			Vector2 v0(5, 5), v1(20, 20);

			auto result = ModificationHelper::FindPolyAndSegmentCrosspoints(v0, v1, &poly, true);

			Assert::IsTrue(result.size() == 0,           L"Must be exactly zero cross points");
		}

		TEST_METHOD(FindPolyAndSegmentCrosspoints__OneEdgeCrosspoint_RayMode)
		{
			auto vertices = new Vector2[4] {
				Vector2(1, 1),
				Vector2(1, 0),
				Vector2(0, 0),
				Vector2(0, 1),
			};
			auto vIDs = new unsigned int[4] { 0, 1, 2, 3 };

			NavMeshPoly poly;
			poly.initialize(4, vIDs);
			poly.vertices = vertices;

			Vector2 v0(.2, .5), v1(.7, .5);

			auto result = ModificationHelper::FindPolyAndSegmentCrosspoints(v0, v1, &poly, true);

			Assert::IsTrue(result.size() == 1,           L"Must be exactly one cross point");
			Assert::IsTrue(result[0] == Vector2(1, 0.5), L"Wrong cross point");
		}

		TEST_METHOD(FindPolyAndSegmentCrosspoints__TwoEdgeCrosspoints_RayMode)
		{
			auto vertices = new Vector2[4] {
				Vector2(1, 1),
				Vector2(1, 0),
				Vector2(0, 0),
				Vector2(0, 1),
			};
			auto vIDs = new unsigned int[4] { 0, 1, 2, 3 };

			NavMeshPoly poly;
			poly.initialize(4, vIDs);
			poly.vertices = vertices;

			Vector2 v0(-1, .5), v1(-.5, .5);

			auto result = ModificationHelper::FindPolyAndSegmentCrosspoints(v0, v1, &poly, true);

			Assert::IsTrue(result.size() == 2, L"Must be exactly two cross points");

			Assert::IsTrue(std::find(result.begin(), result.end(), Vector2(1, 0.5)) != result.end(), L"Missing cross point");
			Assert::IsTrue(std::find(result.begin(), result.end(), Vector2(0, 0.5)) != result.end(), L"Missing cross point");
		}

		TEST_METHOD(FindPolyAndSegmentCrosspoints__OneCornerCrosspoint_RayMode)
		{
			auto vertices = new Vector2[4] {
				Vector2(1, 1),
				Vector2(1, 0),
				Vector2(0, 0),
				Vector2(0, 1),
			};
			auto vIDs = new unsigned int[4] { 0, 1, 2, 3 };

			NavMeshPoly poly;
			poly.initialize(4, vIDs);
			poly.vertices = vertices;

			Vector2 v0(.5, .5), v1(.8, .8);

			auto result = ModificationHelper::FindPolyAndSegmentCrosspoints(v0, v1, &poly, true);

			Assert::IsTrue(result.size() == 1,         L"Must be exactly one cross point");
			Assert::IsTrue(result[0] == Vector2(1, 1), L"Wrong cross point");
		}

		TEST_METHOD(FindPolyAndSegmentCrosspoints__TwoCornerCrosspoints_RayMode)
		{
			auto vertices = new Vector2[4] {
				Vector2(1, 1),
				Vector2(1, 0),
				Vector2(0, 0),
				Vector2(0, 1),
			};
			auto vIDs = new unsigned int[4] { 0, 1, 2, 3 };

			NavMeshPoly poly;
			poly.initialize(4, vIDs);
			poly.vertices = vertices;

			Vector2 v0(-5, -5), v1(-1, -1);

			auto result = ModificationHelper::FindPolyAndSegmentCrosspoints(v0, v1, &poly, true);

			Assert::IsTrue(result.size() == 2, L"Must be exactly two cross points");

			Assert::IsTrue(std::find(result.begin(), result.end(), Vector2(1, 1)) != result.end(), L"Missing cross point");
			Assert::IsTrue(std::find(result.begin(), result.end(), Vector2(0, 0)) != result.end(), L"Missing cross point");
		}

		TEST_METHOD(FindPolyAndSegmentCrosspoints__SegmentOnTheSameLineAsEdge_RayMode)
		{
			auto vertices = new Vector2[4] {
				Vector2(1, 1),
				Vector2(1, 0),
				Vector2(0, 0),
				Vector2(0, 1),
			};
			auto vIDs = new unsigned int[4] { 0, 1, 2, 3 };

			NavMeshPoly poly;
			poly.initialize(4, vIDs);
			poly.vertices = vertices;

			Vector2 v0(-5, 0), v1(-2, 0);

			auto result = ModificationHelper::FindPolyAndSegmentCrosspoints(v0, v1, &poly, true);

			Assert::IsTrue(result.size() == 2, L"Must be exactly two cross points");

			Assert::IsTrue(std::find(result.begin(), result.end(), Vector2(1, 0)) != result.end(), L"Missing cross point");
			Assert::IsTrue(std::find(result.begin(), result.end(), Vector2(0, 0)) != result.end(), L"Missing cross point");
		}

		TEST_METHOD(FindPolyAndSegmentCrosspoints__SegmentOnTheSameLineAsEdgeAndNotCovering_RayMode)
		{
			auto vertices = new Vector2[4] {
				Vector2(1, 1),
				Vector2(1, 0),
				Vector2(0, 0),
				Vector2(0, 1),
			};
			auto vIDs = new unsigned int[4] { 0, 1, 2, 3 };

			NavMeshPoly poly;
			poly.initialize(4, vIDs);
			poly.vertices = vertices;

			Vector2 v0(-1, 0), v1(.5, 0);

			auto result = ModificationHelper::FindPolyAndSegmentCrosspoints(v0, v1, &poly, true);

			Assert::IsTrue(result.size() == 2, L"Must be exactly two cross points");

			Assert::IsTrue(std::find(result.begin(), result.end(), Vector2(.5, 0)) != result.end(), L"Missing cross point");
			Assert::IsTrue(std::find(result.begin(), result.end(), Vector2(1, 0)) != result.end(),  L"Missing cross point");
		}
#pragma endregion
	};
}
