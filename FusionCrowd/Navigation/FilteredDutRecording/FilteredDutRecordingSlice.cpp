#include "FilteredDutRecordingSlice.h"


namespace FusionCrowd
{
	FilteredDutRecordingSlice::FilteredDutRecordingSlice(float time)
	{
		_time = time;
	}


	FilteredDutRecordingSlice::~FilteredDutRecordingSlice()
	{
	}


	float FilteredDutRecordingSlice::GetTime() const {
		return _time;
	}


	size_t FilteredDutRecordingSlice::GetAgentCount() const {
		return m_agentInfos.size();
	}


	AgentInfo FilteredDutRecordingSlice::GetAgentInfo(size_t agentId) const {
		return m_agentInfos.at(agentId).ToAgentInfo();
	}


	void FilteredDutRecordingSlice::GetAgentIds(FCArray<size_t> & outIds) const {
		size_t i = 0;
		for (auto & p : m_agentInfos)
		{
			outIds[i++] = p.first;
		}
	}


	void FilteredDutRecordingSlice::AddAgent(AgentSpatialInfo info)
	{
		m_agentInfos.insert({ info.id, info });
	}
}