#include "RecordingSerializer.h"

#include "Export/IRecordingSlice.h"

#include <fstream>
#include <string>

namespace FusionCrowd
{
	namespace Recordings
	{
		void Serialize(IRecording & rec, char const * destFilePath, size_t pathLen)
		{
			std::string filename(destFilePath, pathLen);
			std::ofstream trajs(filename);

			std::string separator = ",";

			size_t slicesCount = rec.GetSlicesCount();
			TimeSpan timespan(slicesCount);
			rec.GetTimeSpan(timespan);
			for(float step : timespan)
			{
				auto & slice = rec.GetSlice(step);

				auto agentCount = slice.GetAgentCount();
				FCArray<size_t> ids(agentCount);
				slice.GetAgentIds(ids);

				bool first = true;
				for(size_t id : ids)
				{
					auto info = slice.GetAgentInfo(id);
					if(!first)
					{
						trajs << separator;
					}

					trajs << id << separator << info.posX << separator << info.posY << separator << info.orientX << separator << info.orientY;
					first = false;
				}
				trajs << std::endl;
			}
		}
	}
}