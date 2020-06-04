#pragma once

#include "TestCases/ITestCase.h"
#include "Export/ComponentId.h"

namespace TestFusionCrowd
{
	class TradeshowTestCase : public ITestCase
	{
	public:
		TradeshowTestCase(size_t agentsNum, size_t simSteps, bool writeTraj) : ITestCase(agentsNum, simSteps, writeTraj)
		{
		};

		void Pre() override;
		std::string GetName() const override { return "Tradeshow"; };

	private:
		FusionCrowd::ComponentId _op = FusionCrowd::ComponentIds::ORCA_ID;
	};
}
