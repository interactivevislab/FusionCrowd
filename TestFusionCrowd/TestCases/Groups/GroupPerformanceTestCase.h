#pragma once

#include "TestCases/ITestCase.h"

namespace TestFusionCrowd
{
	class GroupPerformanceTestCase : public ITestCase
	{
	public:
		GroupPerformanceTestCase();

		void Pre() override;
		std::string GetName() const override { return "GroupPerformanceTestCase"; };
	};
}
