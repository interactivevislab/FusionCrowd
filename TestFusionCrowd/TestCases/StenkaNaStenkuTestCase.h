#pragma once

#include "TestCases/ITestCase.h"

namespace TestFusionCrowd
{
	class StenkaNaStenkuTestCase : public ITestCase
	{
	public:
		StenkaNaStenkuTestCase(size_t agentsNum, size_t steps, bool writeTraj);

		void Pre() override;
		std::string GetName() const override { return "StenkaNaStenkuTestCase"; };
	};
}
