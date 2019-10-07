#pragma once

#include <memory>

#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/OnlineRecording/OnlineRecordingSlice.h"
#include "Export.h"

#include "Util/FCArray.h"

namespace FusionCrowd
{
	class OnlineRecording : public IRecording
	{
	public:
		OnlineRecording();

		OnlineRecording(OnlineRecording && other);
		OnlineRecording & operator=(OnlineRecording && other);
		~OnlineRecording();


		size_t GetSlicesCount() const override;
		void GetTimeSpan(TimeSpan & outTimeSpan) const override;
		const OnlineRecordingSlice & GetSlice(float time) const override;


		size_t GetAgentCount() const;
		void AddAgent(AgentSpatialInfo spatialInfo);
		bool RemoveAgent(size_t agentId);

		AgentInfo GetAgentInfo(size_t agentId, float time);
		void Update(float timeStep);

		AgentSpatialInfo & GetCurrentSpatialInfo(size_t agentId);
		void GetAgentIds(FCArray<size_t> & outIds);
	private:
		class OnlineRecordingImpl;

		std::unique_ptr<OnlineRecordingImpl> pimpl;
	};
}
