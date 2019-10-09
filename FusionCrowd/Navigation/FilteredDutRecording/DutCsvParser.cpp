#include "DutCsvParser.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>


namespace FusionCrowd
{
	DutCsvParser::DutCsvParser()
	{
	}


	DutCsvParser::~DutCsvParser()
	{
	}

	FilteredDutRecording DutCsvParser::ReadRecordingFromCsv(std::string path) {
		std::ifstream file;
		file.open(path);
		file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

		FilteredDutRecording recording;
		FilteredDutRecordingSlice currentSlice(0);
		float currentFrame = -1;

		while (!file.eof()) {
			auto lineData = ParseLine(file);
			if (currentFrame != lineData.first) {
				if (currentFrame != -1) {
					recording.AddSlice(currentFrame, currentSlice);
				}
				currentFrame = lineData.first;
				currentSlice = FilteredDutRecordingSlice(currentFrame);
			}
			currentSlice.AddAgent(lineData.second);
		}
		recording.AddSlice(currentFrame, currentSlice);

		return recording;
	}

	std::pair<float, AgentSpatialInfo> DutCsvParser::ParseLine(std::ifstream &file) {
		std::string cell;
		std::vector<std::string> cells;
		while (std::getline(file, cell, ','))
		{
			cells.push_back(cell);
		}

		float frame = std::stof(cells[1]);
		AgentSpatialInfo info;
		info.id = std::stoi(cells[0]);
		info.pos.x = std::stof(cells[3]);
		info.pos.y = std::stof(cells[4]);
		info.vel.x = std::stof(cells[5]);
		info.vel.y = std::stof(cells[6]);

		return { frame, info };
	}
}