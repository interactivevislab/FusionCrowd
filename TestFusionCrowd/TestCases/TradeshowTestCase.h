#pragma once

#include "TestCases/ITestCase.h"

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
};
}

