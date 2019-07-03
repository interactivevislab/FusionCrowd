#pragma once

#include "TestCases/ITestCase.h"

#include <string>
#include <vector>

#include "Simulator.h"
#include "Math/Util.h"

namespace TestFusionCrowd
{
	class NeighbourSearchBenchCase : public ITestCase
	{
	public:
		NeighbourSearchBenchCase();

		void Pre() override;
		void Run() override;
		void Post() override;

		~NeighbourSearchBenchCase();

	private:
		float RandFloat(float min, float max);
		void SwitchOperationComponent(FusionCrowd::Simulator & simulator, std::string componentName);
		void AutoSelectOperationComponent(FusionCrowd::Simulator & simulator, int neighborsCount,
			std::string component1Name, std::string component2Name);

	private:
		static const size_t stepsTotal = 4200;
		long long measures[stepsTotal];

		const int input = 10;

		const float goalsDistance = 10;
		const int agentsInGroup = input / 2;
		const int agentsCount = 2 * agentsInGroup;
		const float agentsSpread = 1.f;

		std::vector<std::vector<DirectX::SimpleMath::Vector2>> positions;
		std::vector<DirectX::SimpleMath::Vector2> pointGoals;

		int control1 = 0;
		int control2 = 0;
	};
}
