#pragma once

#include "TestCases/ITestCase.h"

#include <string>
#include <vector>
#include <memory>

#include "Math/Util.h"
#include "Export/IRecording.h"

namespace TestFusionCrowd
{
	class ZanlungoCase : public ITestCase
	{
	public:
		ZanlungoCase();

		void Pre() override;
		void Run() override;
		void Post() override;
		std::string GetName() const override { return "Zanlungo"; };

		~ZanlungoCase();

	private:
		static const size_t stepsTotal = 5000;
		long long measures[stepsTotal];

		const float worldSide = 20;
		const int totalAgents = 10;

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

