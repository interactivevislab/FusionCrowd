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
		OnlineRecordingSlice(float time);
		OnlineRecordingSlice(const OnlineRecordingSlice & other);
		OnlineRecordingSlice(std::map<size_t, AgentSpatialInfo> agentsSpatialInfos, float newTime);
		OnlineRecordingSlice(OnlineRecordingSlice && other);
		OnlineRecordingSlice& operator=(const OnlineRecordingSlice & other);
		OnlineRecordingSlice& operator=(OnlineRecordingSlice && other);

		float GetTime() const override;
		size_t GetAgentCount() const override;
		AgentInfo GetAgentInfo(size_t agentId) const override;
		void GetAgentIds(FCArray<size_t> & outIds) const override;

		AgentSpatialInfo & GetInfo(size_t agentId);
		void AddAgent(AgentSpatialInfo info);
		bool RemoveAgent(size_t agentId);

	private:
		float _time;
		std::map<size_t, AgentSpatialInfo> m_agentInfos;
	};
}
