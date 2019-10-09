#pragma once

#include <memory>
#include <vector>

#include "Navigation/FilteredDutRecording/FilteredDutRecordingSlice.h"

namespace FusionCrowd
{
	class FilteredDutRecording : public IRecording
	{
	public:
		FilteredDutRecording();
		~FilteredDutRecording();

		size_t GetSlicesCount() const override;
		void GetTimeSpan(TimeSpan & outTimeSpan) const override;
		const IRecordingSlice & GetSlice(float time) const override;

		static FilteredDutRecording ReadFromCsv(std::string path);
		void AddSlice(float frame, FilteredDutRecordingSlice slice);

	private:
		/*class FilteredDutRecordingImpl;

		std::unique_ptr<FilteredDutRecordingImpl> pimpl;*/

		std::vector<std::pair<float, FilteredDutRecordingSlice>> _recordingData;
	};
}