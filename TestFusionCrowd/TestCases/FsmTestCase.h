#pragma once

#include "TestCases/ITestCase.h"
#include "Export/ComponentId.h"
#include "Export/Export.h"

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
		void Run() override;
		void Post() override;

		std::string GetName() const override { return "fsm"; };

	private:
		std::unique_ptr<FusionCrowd::ISimulatorFacade, decltype(&FusionCrowd::SimulatorFacadeDeleter)> _sim;

		FusionCrowd::ComponentId _opComponent;
		const size_t _agentsNum;
		const size_t _simulationSteps;
		const bool _writeTrajectories;
		std::chrono::time_point<std::chrono::system_clock> _startTime;
		std::vector<long long> _measures;
	};
}
