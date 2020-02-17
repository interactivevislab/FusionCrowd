#include "stdafx.h"
#include "CppUnitTest.h"
#include "resources_util.h"

#include <fstream>

#include "Navigation/NavMesh/NavMesh.h"


using namespace Microsoft::VisualStudio::CppUnitTestFramework;

using namespace FusionCrowd;

namespace UnitTest
{
	TEST_CLASS(NavMeshUnitTest)
	{
	public:
		TEST_METHOD(NavMesh__Load)
		{
			std::ifstream f(GetDirectoryName(__FILE__) + "square.nav");
			auto navMesh = NavMesh::Load(f);


			Assert::IsFalse(nullptr == navMesh.get(), L"NavMesh couldn't load.");
			Assert::IsTrue(1 == navMesh->GetNodesCount());
			Assert::IsTrue(0 == navMesh->GetEdgesCount());
			Assert::IsTrue(4 == navMesh->GetVertexCount());
		}
	};
}