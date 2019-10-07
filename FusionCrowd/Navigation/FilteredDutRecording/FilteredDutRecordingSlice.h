#pragma once

#include <map>

#include "Util/IRecording.h"
#include "Navigation/AgentSpatialInfo.h"
#include "Export.h"


namespace FusionCrowd
{
	class FilteredDutRecordingSlice : public IRecordingSlice
	{
	public:
		FilteredDutRecordingSlice();
		~FilteredDutRecordingSlice();

		size_t GetAgentCount() const override;
		AgentInfo GetAgentInfo(size_t agentId) const override;
		void GetAgentIds(FCArray<size_t> & outIds) const override;

		void AddAgent(AgentSpatialInfo info);

	private:
		std::map<size_t, AgentSpatialInfo> m_agentInfos;
	};
}