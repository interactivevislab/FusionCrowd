#pragma once

#include <random>

namespace TestFusionCrowd
{
	namespace
	{
		std::mt19937 e2(10);
	}

	inline float RandFloat(float min, float max)
	{
		std::uniform_real_distribution<> dist(min, max);
		return dist(e2);
	}
}