#pragma once

#include "TestCases/ITestCase.h"
#include "Export/ComponentId.h"
#include "Export/Export.h"

#include <memory>
#include <vector>
#include <chrono>

namespace TestFusionCrowd
{
	class TshapedFancyTestCase : public ITestCase
	{
	public:
		TshapedFancyTestCase(
			FusionCrowd::ComponentId opComponent=FusionCrowd::ComponentIds::KARAMOUZAS_ID,
			size_t agentsNum=10000,
			size_t simulationSteps=5000,
			bool writeTraj=false
		);

		void Pre() override;
		std::string GetName() const override { return "TshapedFancyTestCase"; };

	private:
		FusionCrowd::ComponentId _opComponent;
	};
}
