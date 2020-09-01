#pragma once

#include "Export/Config.h"
#include "Export/FCArray.h"

#include <vector>


namespace FusionCrowd
{
	template<typename T>
	static FCArray<T> VectorToFcArray(const std::vector<T>& inVector)
	{
		FCArray<T> output(inVector.size());
		for (int i = 0; i < output.size(); i++)
		{
			output[i] = inVector[i];
		}
		return std::move(output);
	}
}
