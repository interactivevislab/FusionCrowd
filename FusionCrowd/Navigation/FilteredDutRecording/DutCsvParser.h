#pragma once

#include "FilteredDutRecording.h"

namespace FusionCrowd
{
	class DutCsvParser
	{
	public:
		static FilteredDutRecording ReadRecordingFromCsv(std::string path);
	private:
		DutCsvParser();
		~DutCsvParser();

		static std::pair<float, AgentSpatialInfo> ParseLine(std::ifstream &file);
	};
}