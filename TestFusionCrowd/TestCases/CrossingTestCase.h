#pragma once

#include "TestCases/ITestCase.h"
#include "Export/ComponentId.h"
#include "Export/Export.h"

#include <memory>
#include <vector>
#include <chrono>

namespace TestFusionCrowd
{
	class CrossingTestCase : public ITestCase
	{
	public:
		CrossingTestCase(
			FusionCrowd::ComponentId opComponent=FusionCrowd::ComponentIds::KARAMOUZAS_ID,
			size_t agentsNum=10000,
			size_t simulationSteps=5000,
			bool writeTrajectories=false
		);

		void Pre() override;
		std::string GetName() const override { return "Crossing"; };

	private:
		FusionCrowd::ComponentId _opComponent;
	};
}
