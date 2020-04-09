#pragma once

#include "TestCases/ITestCase.h"

#include "Export/ComponentId.h"

namespace TestFusionCrowd
{
	class ExchangeCircleCase : public ITestCase
	{
	public:
		ExchangeCircleCase(size_t agentsNum, size_t steps, FusionCrowd::ComponentId op, bool writeTraj);

		void Pre() override;
		std::string GetName() const override { return "ExchangeCircleCase"; };
	private:
		FusionCrowd::ComponentId _op;
	};
}
