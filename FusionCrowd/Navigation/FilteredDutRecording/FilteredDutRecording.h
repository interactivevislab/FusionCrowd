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

		TimeSpan GetTimeSpan() const override;
		const FilteredDutRecordingSlice & GetSlice(float time) const override;

		static FilteredDutRecording ReadFromCsv(std::string path);
		void AddSlice(float frame, FilteredDutRecordingSlice slice);

	private:
		/*class FilteredDutRecordingImpl;

		std::unique_ptr<FilteredDutRecordingImpl> pimpl;*/

		std::vector<std::pair<float, FilteredDutRecordingSlice>> _recordingData;
	};
}