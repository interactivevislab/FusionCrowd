// TestFusionCrowd.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include "pch.h"
#include <iostream>

#include "TestCases/NeighbourSearchBenchCase.h"

using namespace TestFusionCrowd;

int main()
{
	NeighbourSearchBenchCase case1;

	case1.Pre();

	std::cout << "Overclocking launch... ";
	case1.Run(3.0f);
	std::cout << "complete" << std::endl;

	/*for (float coeff = 0.5; coeff < 10; coeff += 0.25) {
		std::cout << "coeff = " << coeff << '\t';
		case1.Run(coeff);
		case1.Post();
	}
	*/
}
