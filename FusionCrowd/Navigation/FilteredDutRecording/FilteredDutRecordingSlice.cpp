#include "FilteredDutRecordingSlice.h"


namespace FusionCrowd
{
	FilteredDutRecordingSlice::FilteredDutRecordingSlice()
	{
	}


	FilteredDutRecordingSlice::~FilteredDutRecordingSlice()
	{
	}


	size_t FilteredDutRecordingSlice::GetAgentCount() const {
		return m_agentInfos.size();
	}


	AgentInfo FilteredDutRecordingSlice::GetAgentInfo(size_t agentId) const {
		return m_agentInfos.at(agentId).ToAgentInfo();
	}


	FCArray<size_t> FilteredDutRecordingSlice::GetAgentIds() const {
		FCArray<size_t> ids(m_agentInfos.size());

		size_t i = 0;
		for (auto & p : m_agentInfos)
		{
			ids[i] = p.first;
			i++;
		}

		return ids;
	}


	void FilteredDutRecordingSlice::AddAgent(AgentSpatialInfo info)
	{
		m_agentInfos.insert({ info.id, info });
	}
}