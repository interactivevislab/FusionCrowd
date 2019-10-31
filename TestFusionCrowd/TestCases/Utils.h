#pragma once

#include <random>

namespace TestFusionCrowd
{
	namespace
	{
		std::random_device rd;
		std::default_random_engine e2(rd());
	}

	inline float RandFloat(float min, float max)
	{
		std::uniform_real_distribution<> dist(min, max);
		return dist(e2);
	}
}