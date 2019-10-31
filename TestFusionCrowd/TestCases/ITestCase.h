#pragma once

#include "Export/Export.h"

#include <string>
#include <memory>

namespace TestFusionCrowd
{
	class ITestCase
	{
	public:
		ITestCase() { }
		ITestCase(size_t agentsNum, size_t simSteps) : _agentsNum(agentsNum), _simulationSteps(simSteps) { }
		ITestCase(size_t agentsNum, size_t simSteps, bool writeTraj)
		: _agentsNum(agentsNum), _simulationSteps(simSteps), WriteTrajectories(writeTraj)
		{ }
		ITestCase(size_t agentsNum, size_t simSteps, bool verboseSteps, bool writeTime, bool writeTraj)
		: _agentsNum(agentsNum), _simulationSteps(simSteps), VerboseSteps(verboseSteps), WriteTrajectories(writeTraj), WriteTime(writeTime)
		{ }

		virtual void Pre() = 0;
		virtual void Post() { };
		virtual std::string GetName() const = 0;

		size_t GetStepCount() const { return _simulationSteps; };
		size_t GetAgentCount() const { return _agentsNum; };
		std::shared_ptr<FusionCrowd::ISimulatorFacade> GetSim() const { return _sim; };

		virtual ~ITestCase() { };

	public:
		const bool VerboseSteps      = false;
		const bool WriteTrajectories = false;
		const bool WriteTime         = true;

	protected:
		const size_t _agentsNum       = 100;
		const size_t _simulationSteps = 1000;
		std::shared_ptr<FusionCrowd::ISimulatorFacade> _sim;
	};
}
