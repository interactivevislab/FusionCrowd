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
			virtual void GetAgentIds(FCArray<size_t> & outIds) const = 0;

			virtual ~IRecordingSlice() { }
		};
	}
}
