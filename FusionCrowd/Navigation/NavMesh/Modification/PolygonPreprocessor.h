#pragma once
#include <vector>
#include "Export/Export.h"
#include "Math/Util.h"

using namespace DirectX::SimpleMath;

namespace FusionCrowd {
	class ModificationProcessor;

	struct Poly {
		std::vector<Vector2> v;

		bool isReflex(const int &i);
		bool canSee(const int &a, const int &b);
		Poly* copy(const int &i, const int &j);
		Vector2& at(int i);
	};

	class PolygonPreprocessor
	{
	public:
		PolygonPreprocessor(FCArray<NavMeshVetrex> & polygon);
		~PolygonPreprocessor();
		float performAll(ModificationProcessor& processor);

	private:
		void clearRes(std::vector<Poly*>& res);
		Poly* input_poly;
		std::vector<Poly*> decomp(Poly* poly);
	};
}

