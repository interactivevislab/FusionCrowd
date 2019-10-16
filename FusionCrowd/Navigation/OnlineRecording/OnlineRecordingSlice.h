#pragma once

#include <map>

#include "Export/IRecording.h"
#include "Export/FCArray.h"
#include "Export/Export.h"
#include "Navigation/AgentSpatialInfo.h"

namespace FusionCrowd
{
	class OnlineRecordingSlice : public IRecordingSlice
	{
	public:
		OnlineRecordingSlice(float time);
		OnlineRecordingSlice(FCArray<AgentInfo> agentsInfos, float newTime);
		OnlineRecordingSlice(const OnlineRecordingSlice & other);
		OnlineRecordingSlice(OnlineRecordingSlice && other);

		OnlineRecordingSlice& operator=(const OnlineRecordingSlice & other);
		OnlineRecordingSlice& operator=(OnlineRecordingSlice && other);

		float GetTime() const override;
		size_t GetAgentCount() const override;
		AgentInfo GetAgentInfo(size_t agentId) const override;
		void GetAgentIds(FCArray<size_t> & outIds) const override;

		void AddAgent(AgentInfo info);
		bool RemoveAgent(size_t agentId);

	private:
		float _time;
		std::map<size_t, AgentInfo> m_agentInfos;
	};
}
