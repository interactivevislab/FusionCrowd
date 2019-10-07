#include "FilteredDutRecording.h"
#include "DutCsvParser.h"

#include <vector>
#include <algorithm>

namespace FusionCrowd
{
	//class FilteredDutRecording::FilteredDutRecordingImpl {
	//public:
	//	TimeSpan GetTimeSpan() const {
	//		TimeSpan timeSpan(_recordingData.size);
	//		auto target = timeSpan.begin();
	//		for (auto it = _recordingData.begin(); it != _recordingData.end(); it++) {
	//			*target = it->first;
	//			target++;
	//		}
	//		return timeSpan;
	//	}

	//	const FilteredDutRecordingSlice & GetSlice(float time) const {
	//		assert(time >= 0 && "Time must be non-negative");

	//		auto it = std::next(_recordingData.begin());
	//		while ((it != _recordingData.end()) && (it->first < time)) {
	//			it++;
	//		}

	//		return std::prev(_recordingData.begin())->second;
	//	}

	//	static FilteredDutRecording ReadFromCsv(std::string path) {
	//		return DutCsvParser::ReadRecordingFromCsv(path);
	//	}

	//	void AddSlice(float frame, FilteredDutRecordingSlice slice) {
	//		_recordingData.push_back({ frame, slice });
	//	}

	//private:
	//	std::vector<std::pair<float, FilteredDutRecordingSlice>> _recordingData;
	//};

	FilteredDutRecording::FilteredDutRecording()
	{
	}


	FilteredDutRecording::~FilteredDutRecording()
	{
	}


	size_t FilteredDutRecording::GetSlicesCount() const {
		return _recordingData.size();
	}


	void FilteredDutRecording::GetTimeSpan(TimeSpan & outTimeSpan) const {
		auto target = outTimeSpan.begin();
		for (auto it = _recordingData.begin(); it != _recordingData.end(); it++) {
			*target = it->first;
			target++;
		}
	}


	const FilteredDutRecordingSlice & FilteredDutRecording::GetSlice(float time) const {
		assert(time >= 0 && "Time must be non-negative");

		auto it = std::next(_recordingData.begin());
		while ((it != _recordingData.end()) && (it->first < time)) {
			it++;
		}

		return std::prev(_recordingData.begin())->second;
	}

	FilteredDutRecording FilteredDutRecording::ReadFromCsv(std::string path) {
		return DutCsvParser::ReadRecordingFromCsv(path);
	}

	void FilteredDutRecording::AddSlice(float frame, FilteredDutRecordingSlice slice) {
		_recordingData.push_back({ frame, slice });
	}
}