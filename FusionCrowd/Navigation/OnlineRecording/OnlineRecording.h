#pragma once

#include <memory>

#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/OnlineRecording/OnlineRecordingSlice.h"
#include "Util/PublicSpatialInfo.h"

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


		TimeSpan * GetTimeSpan() const override;
		const OnlineRecordingSlice & GetSlice(float time) const override;


		size_t GetAgentCount() const;
		void AddAgent(AgentSpatialInfo spatialInfo);
		bool RemoveAgent(size_t agentId);

		PublicSpatialInfo GetPublicSpatialInfo(size_t agentId, float time);
		void Update(float timeStep);

		AgentSpatialInfo & GetCurrentSpatialInfo(size_t agentId);
		FCArray<size_t> GetAgentIds();
	private:
		class OnlineRecordingImpl;

		std::unique_ptr<OnlineRecordingImpl> pimpl;
	};
}
