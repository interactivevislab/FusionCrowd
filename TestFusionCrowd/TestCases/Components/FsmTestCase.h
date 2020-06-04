#pragma once

#include "TestCases/ITestCase.h"
#include "Export/ComponentId.h"

#include <memory>
#include <chrono>
#include <vector>

namespace TestFusionCrowd
{
	class FsmTestCase : ITestCase
	{
	public:
		FsmTestCase(
			FusionCrowd::ComponentId opComponent=FusionCrowd::ComponentIds::KARAMOUZAS_ID,
			size_t agentsNum=100,
			size_t simulationSteps=5000,
			bool writeTrajectories=false
		);

		void Pre() override;

		std::string GetName() const override { return "fsm"; };

	private:
		FusionCrowd::ComponentId _opComponent;
	};
}
