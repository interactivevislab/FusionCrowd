#pragma once

#include "Config.h"

namespace FusionCrowd
{
	class FUSION_CROWD_API TimeSpan
	{
	public:
		TimeSpan(const float* vals, size_t l);

		const float* values;
		const size_t len;

		~TimeSpan();
	};
}
