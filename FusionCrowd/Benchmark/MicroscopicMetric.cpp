#include "MicroscopicMetrics.h"

#include <math.h>

float FusionCrowd::MicroscopicMetrics::AbsoluteDifference(const IRecording & rec1, const IRecording & rec2)
{
	float result = 0;

	auto ts = rec1.GetTimeSpan();

	size_t remaining = ts.len;
	while(remaining --> 0)
	{
		float time = ts.vals[remaining];

		auto & slice1 = rec1.GetSlice(time);
		auto & slice2 = rec2.GetSlice(time);

		for(size_t id : slice1.GetAgentIds())
		{
			auto a1 = slice1.GetAgentInfo(id);
			auto a2 = slice2.GetAgentInfo(id);
			result += sqrtf(powf(a1.posX - a2.posX, 2.0f) + powf(a1.posY - a2.posY, 2.0f));
		}
	}

	return result;
}