#pragma once

#include <memory>

#include "Navigation/AgentSpatialInfo.h"
#include "Navigation/OnlineRecording/OnlineRecordingSlice.h"
#include "Export/Export.h"

#include "Export/FCArray.h"

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
		const OnlineRecordingSlice & GetCurrentSlice() const override;
		const IRecordingSlice * Begin() const override;
		const IRecordingSlice * End() const override;
		bool LoadFromFile(char const * path, size_t path_length) override;
		void Serialize(char const * destFilePath, size_t pathLen) const override;

		size_t GetAgentCount() const;

		void MakeRecord(FCArray<AgentInfo> agentsInfos, float timeStep);

		void GetAgentIds(FCArray<size_t> & outIds);
	private:
		class OnlineRecordingImpl;

		std::unique_ptr<OnlineRecordingImpl> pimpl;
	};
}
