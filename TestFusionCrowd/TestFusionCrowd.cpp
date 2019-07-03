// TestFusionCrowd.cpp : This file contains the 'main' function. Program execution begins and ends there.

#include "pch.h"

#include "TestCases/NeighbourSearchBenchCase.h"

using namespace TestFusionCrowd;

int main()
{
	NeighbourSearchBenchCase case1;

	case1.Pre();

	for (int i = 0; i < 5; i++)
		case1.Run();

	case1.Post();
}
