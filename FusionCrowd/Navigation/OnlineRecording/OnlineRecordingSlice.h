#pragma once

#include <map>

#include "Util/IRecording.h"
#include "Util/PublicSpatialInfo.h"
#include "Navigation/AgentSpatialInfo.h"

namespace FusionCrowd
{
	class OnlineRecordingSlice : public IRecordingSlice
	{
	public:
		OnlineRecordingSlice();

		size_t GetAgentCount() const;
		PublicSpatialInfo GetAgentInfo(size_t agentId) const;

		AgentSpatialInfo & GetInfo(size_t agentId);
		void AddAgent(AgentSpatialInfo info);
		bool RemoveAgent(size_t agentId);

	private:
		std::map<size_t, AgentSpatialInfo> m_agentInfos;
	};
}
