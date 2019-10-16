// TestFusionCrowd.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include "pch.h"
#include <iostream>

#include "TestCases/NeighbourSearchBenchCase.h"
#include "TestCases/ZanlungoCase.h"
#include "TestCases/CrossingTestCase.h"
#include "TestCases/PinholeTestCase.h"
#include "TestCases/TshapedFancyTestCase.h"

#include "ComponentId.h"

using namespace TestFusionCrowd;

int main()
{
	NeighbourSearchBenchCase case1;
	ZanlungoCase case2;
	CrossingTestCase crossingCase(FusionCrowd::ComponentIds::KARAMOUZAS_ID, 30, 1000, true);
	PinholeTestCase pinholeCase(FusionCrowd::ComponentIds::KARAMOUZAS_ID, 200, 1000, true);
	TshapedFancyTestCase tshapedCase(FusionCrowd::ComponentIds::KARAMOUZAS_ID, 200, 1000, true);

	crossingCase.Pre();
	crossingCase.Run(0);
	crossingCase.Post();

	pinholeCase.Pre();
	pinholeCase.Run(0);
	pinholeCase.Post();

	tshapedCase.Pre();
	tshapedCase.Run(0);
	tshapedCase.Post();


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
