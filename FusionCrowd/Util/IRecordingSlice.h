#pragma once

#include "Config.h"
#include "Util/FCArray.h"
#include "Util/PublicSpatialInfo.h"

namespace FusionCrowd
{
	class FUSION_CROWD_API IRecordingSlice
	{
	public:
		virtual size_t GetAgentCount() const = 0;
		virtual PublicSpatialInfo GetAgentInfo(size_t agentId) const = 0;
		virtual FCArray<size_t> GetAgentIds() const = 0;

		virtual ~IRecordingSlice() { }
	};
}
