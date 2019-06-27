#include "OnlineRecordingSlice.h"

namespace FusionCrowd
{
	OnlineRecordingSlice::OnlineRecordingSlice()
	{
	}

	size_t OnlineRecordingSlice::GetAgentCount() const
	{
		return m_agentInfos.size();
	};

	PublicSpatialInfo OnlineRecordingSlice::GetAgentInfo(size_t agentId) const
	{
		return m_agentInfos.at(agentId).ToPublicInfo();
	};

	AgentSpatialInfo& OnlineRecordingSlice::GetInfo(size_t agentId)
	{
		return m_agentInfos[agentId];
	};

	void OnlineRecordingSlice::AddAgent(AgentSpatialInfo info)
	{
		m_agentInfos.insert({info.id, info});
	}

	bool OnlineRecordingSlice::RemoveAgent(size_t agentId)
	{
		return m_agentInfos.erase(agentId) > 0;
	}
}
