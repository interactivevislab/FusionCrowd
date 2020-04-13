#pragma once

#include "TestCases/ITestCase.h"

namespace TestFusionCrowd
{
	class GroupMovementTestCase : public ITestCase
	{
	public:
		GroupMovementTestCase(size_t steps, bool writeTraj);

		void Pre() override;
		std::string GetName() const override { return "GroupMovementTestCase"; };
	};
}
