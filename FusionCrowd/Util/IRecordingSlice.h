#pragma once

#include "Config.h"
#include "Util/FCArray.h"
#include "Export.h"

namespace FusionCrowd
{
	struct AgentInfo;

	extern "C"
	{
		class FUSION_CROWD_API IRecordingSlice
		{
		public:
			virtual size_t GetAgentCount() const = 0;
			virtual AgentInfo GetAgentInfo(size_t agentId) const = 0;
			virtual FCArray<size_t> GetAgentIds() const = 0;

			virtual ~IRecordingSlice() { }
		};
	}
}
