#pragma once

#include "TestCases/ITestCase.h"
#include "Export/ComponentId.h"
#include "Export/Export.h"

#include <memory>
#include <vector>
#include <chrono>

namespace TestFusionCrowd
{
	class TshapedFancyTestCase : public ITestCase<float>
	{
	public:
		TshapedFancyTestCase(
			FusionCrowd::ComponentId opComponent=FusionCrowd::ComponentIds::KARAMOUZAS_ID,
			size_t agentsNum=10000,
			size_t simulationSteps=5000,
			bool writeTrajectories=false
		);

		void Pre() override;
		void Run(const float& args) override;
		void Post() override;

	private:
		float RandFloat(float min, float max);

		FusionCrowd::ComponentId _opComponent;
		std::unique_ptr<FusionCrowd::ISimulatorFacade, decltype(&FusionCrowd::SimulatorFacadeDeleter)> _sim;

		const size_t _agentsNum;
		const size_t _simulationSteps;
		const bool _writeTrajectories;
		std::chrono::time_point<std::chrono::system_clock> _startTime;
		std::vector<long long> _measures;
	};
}
