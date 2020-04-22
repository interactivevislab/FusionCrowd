#pragma once

#include "TestCases/ITestCase.h"

#include <string>
#include <vector>
#include <memory>

#include "Math/Util.h"
#include "Export/IRecording.h"
#include "Export/Export.h"

namespace TestFusionCrowd
{
	class ZanlungoCase : public ITestCase
	{
	public:
		ZanlungoCase();

		void Pre() override;
		std::string GetName() const override { return "Zanlungo"; };

	private:
		const float worldSide = 20;
		const int totalAgents = 10;

		const float goalsDistance = 10;
		const int agentsInGroup = totalAgents / 2;
		const int agentsCount = 2 * agentsInGroup;
		const float agentsSpread = 1.f;
		const float searchRadius = 5;

		int control1 = 0;
		int control2 = 0;
	};
}

