#pragma once

#include "TestCases/ITestCase.h"
#include "Export/ComponentId.h"
#include "Export/Export.h"

#include <memory>
#include <vector>
#include <chrono>

namespace TestFusionCrowd
{
	class NavGraphTestCase : public ITestCase
	{
	public:
		NavGraphTestCase(
			//FusionCrowd::ComponentId opComponent = FusionCrowd::ComponentIds::ORCA_ID,
			size_t agentsNum = 3,
			size_t simulationSteps = 100,
			bool writeTraj = true
		);

		void Pre() override;
		std::string GetName() const override { return "NavGraphTest"; };

	private:
		FusionCrowd::ComponentId _opComponent;
	};
}

