#pragma once

#include "TestCases/ITestCase.h"

#include <string>
#include <vector>
#include <memory>

#include "Simulator.h"
#include "Math/Util.h"
#include "Util/IRecording.h"

namespace TestFusionCrowd
{
	class NeighbourSearchBenchCase : public ITestCase<float>
	{
	public:
		NeighbourSearchBenchCase();

		void Pre() override;
		void Run(const float & coef) override;
		void Post() override;

		~NeighbourSearchBenchCase();

	private:
		float RandFloat(float min, float max);

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
	};
}
