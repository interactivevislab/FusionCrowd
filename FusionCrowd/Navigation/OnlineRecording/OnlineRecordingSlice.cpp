#include "OnlineRecordingSlice.h"

namespace FusionCrowd
{
	OnlineRecordingSlice::OnlineRecordingSlice(float time)
	{
		_time = time;
	}

	float OnlineRecordingSlice::GetTime() const {
		return _time;
	}

	size_t OnlineRecordingSlice::GetAgentCount() const
	{
		return m_agentInfos.size();
	};

	AgentInfo OnlineRecordingSlice::GetAgentInfo(size_t agentId) const
	{
		return m_agentInfos.at(agentId);
	};

	void OnlineRecordingSlice::AddAgent(AgentInfo info)
	{
		m_agentInfos.insert({info.id, info});
	}

	bool OnlineRecordingSlice::RemoveAgent(size_t agentId)
	{
		return m_agentInfos.erase(agentId) > 0;
	}

	void OnlineRecordingSlice::GetAgentIds(FCArray<size_t> & outIds) const
	{
		size_t i = 0;
		for(auto & p : m_agentInfos)
		{
			outIds[i++] = p.first;
		}
	}

	OnlineRecordingSlice::OnlineRecordingSlice(const OnlineRecordingSlice & other) = default;
	OnlineRecordingSlice::OnlineRecordingSlice(OnlineRecordingSlice && other) = default;
	OnlineRecordingSlice& OnlineRecordingSlice::operator=(const OnlineRecordingSlice & other) = default;
	OnlineRecordingSlice& OnlineRecordingSlice::operator=(OnlineRecordingSlice && other) = default;

	OnlineRecordingSlice::OnlineRecordingSlice(FCArray<AgentInfo> agentsInfos, float newTime) {
		for (auto info : agentsInfos) {
			m_agentInfos.insert({ info.id, info });
		}
		_time = newTime;
	}
}
