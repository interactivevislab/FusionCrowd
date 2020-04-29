#pragma once

#include "TestCases/ITestCase.h"

namespace TestFusionCrowd
{
	class GoalShapeTestCase : public ITestCase
	{
	public:
		GoalShapeTestCase();

		void Pre() override;
		std::string GetName() const override { return "GoalShapeTestCase"; };

	private:
		void SetupOutOfBoundsPoint();
		void SetupPointShape();
		void SetupDiskShape();
		void SetupRectShape();
	};
}
