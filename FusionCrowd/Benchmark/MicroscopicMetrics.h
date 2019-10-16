#pragma once

#include "Export/Config.h"
#include "Export/IRecording.h"

namespace FusionCrowd
{
	namespace MicroscopicMetrics
	{
		FUSION_CROWD_API float AbsoluteDifference(const IRecording & rec1, const IRecording & rec2);
		FUSION_CROWD_API float PathLength(IRecording & rec1, IRecording & rec2);
		FUSION_CROWD_API float InnerPedestrianDistance(IRecording & rec1, IRecording & rec2);
		FUSION_CROWD_API float ProgressiveDistance(IRecording & rec1, IRecording & rec2);
	};
}
