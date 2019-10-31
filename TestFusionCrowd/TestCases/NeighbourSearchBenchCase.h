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
		void Run() override;
		void Post() override;

		std::string GetName() const override { return "NeighbourSearch"; };

		~NeighbourSearchBenchCase();

	private:
		static const size_t stepsTotal = 100;
		long long measures[stepsTotal];

		const float worldSide = 100;
		const int totalAgents = 5 * worldSide * worldSide;

		const float goalsDistance = 10;
		const int agentsInGroup = totalAgents / 2;
		const int agentsCount = 2 * agentsInGroup;
		const float agentsSpread = 1.f;
		const float searchRadius = 5;

		std::vector<std::vector<DirectX::SimpleMath::Vector2>> positions;
		std::vector<DirectX::SimpleMath::Vector2> pointGoals;
		FusionCrowd::IRecording* recording;

		int control1 = 0;
		int control2 = 0;
		float _coef;
	};
}
