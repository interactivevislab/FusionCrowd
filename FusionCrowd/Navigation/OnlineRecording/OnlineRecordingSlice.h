#pragma once

#include <map>

#include "Util/IRecording.h"
#include "Util/FCArray.h"
#include "Export.h"
#include "Navigation/AgentSpatialInfo.h"

namespace FusionCrowd
{
	class OnlineRecordingSlice : public IRecordingSlice
	{
	public:
		OnlineRecordingSlice();
		OnlineRecordingSlice(const OnlineRecordingSlice & other);
		OnlineRecordingSlice(OnlineRecordingSlice && other);
		OnlineRecordingSlice& operator=(const OnlineRecordingSlice & other);
		OnlineRecordingSlice& operator=(OnlineRecordingSlice && other);

		size_t GetAgentCount() const override;
		AgentInfo GetAgentInfo(size_t agentId) const override;
		FCArray<size_t> GetAgentIds() const override;

		AgentSpatialInfo & GetInfo(size_t agentId);
		void AddAgent(AgentSpatialInfo info);
		bool RemoveAgent(size_t agentId);

	private:
		std::map<size_t, AgentSpatialInfo> m_agentInfos;
	};
}
