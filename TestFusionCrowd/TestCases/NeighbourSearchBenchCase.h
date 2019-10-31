#pragma once

#include "TestCases/ITestCase.h"

#include <string>
#include <vector>
#include <memory>

#include "Math/Util.h"
#include "Export/IRecording.h"

namespace TestFusionCrowd
{
	class NeighbourSearchBenchCase : public ITestCase
	{
	public:
		NeighbourSearchBenchCase(float coef);

		void Pre() override;
		std::string GetName() const override { return "NeighbourSearch"; };

	private:
		const float worldSide = 100;

		const float goalsDistance = 10;
		const int agentsInGroup = _agentsNum / 2;
		const int agentsCount = 2 * agentsInGroup;
		const float agentsSpread = 1.f;
		const float searchRadius = 5;

		int control1 = 0;
		int control2 = 0;
		float _coef;
	};
}
