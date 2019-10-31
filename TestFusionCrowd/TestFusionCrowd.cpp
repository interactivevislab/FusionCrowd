// TestFusionCrowd.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include "pch.h"
#include <iostream>

#include "TestCases/NeighbourSearchBenchCase.h"
#include "TestCases/ZanlungoCase.h"
#include "TestCases/CrossingTestCase.h"
#include "TestCases/PinholeTestCase.h"
#include "TestCases/TshapedFancyTestCase.h"
#include "TestCases/FsmTestCase.h"
#include "TestCases/ITestCase.h"

#include "Export/ComponentId.h"

using namespace TestFusionCrowd;

int main()
{
	NeighbourSearchBenchCase case1(0.5f);
	ZanlungoCase case2;
	CrossingTestCase crossingCase(FusionCrowd::ComponentIds::KARAMOUZAS_ID, 30, 1000, false);
	PinholeTestCase pinholeCase(FusionCrowd::ComponentIds::KARAMOUZAS_ID, 200, 1000, false);
	TshapedFancyTestCase tshapedCase(FusionCrowd::ComponentIds::ZANLUNGO_ID, 200, 1000, false);
	FsmTestCase fsmTestCase(FusionCrowd::ComponentIds::GCF_ID, 50, 2000, true);

	/*crossingCase.Pre();
	crossingCase.Run(0);
	crossingCase.Post();

	pinholeCase.Pre();
	pinholeCase.Run(0);
	pinholeCase.Post();

	tshapedCase.Pre();
	tshapedCase.Run(0);
	tshapedCase.Post();*/

	fsmTestCase.Pre();
	fsmTestCase.Run();
	fsmTestCase.Post();

	std::vector<std::shared_ptr<ITestCase>> cases;

	for(auto testCase : cases)
	{

	}

	/*
	case1.Pre();
	case2.Pre();

	std::cout << "Overclocking launch... ";
	case1.Run(3.0f);
	case2.Run(3.0f);
	std::cout << "complete" << std::endl;

	for (float coeff = 0.5; coeff < 10; coeff += 0.25) {
		std::cout << "coeff = " << coeff << '\t';
		case1.Run(coeff);
		case1.Post();
	}
	*/
}
