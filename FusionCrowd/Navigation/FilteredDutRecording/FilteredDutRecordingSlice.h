#pragma once

#include <map>

#include "Util/IRecording.h"
#include "Navigation/AgentSpatialInfo.h"


namespace FusionCrowd
{
	class FilteredDutRecordingSlice : public IRecordingSlice
	{
	public:
		FilteredDutRecordingSlice();
		~FilteredDutRecordingSlice();

		size_t GetAgentCount() const override;
		PublicSpatialInfo GetAgentInfo(size_t agentId) const override;
		FCArray<size_t> GetAgentIds() const override;

		void AddAgent(AgentSpatialInfo info);

	private:
		std::map<size_t, AgentSpatialInfo> m_agentInfos;
	};
}