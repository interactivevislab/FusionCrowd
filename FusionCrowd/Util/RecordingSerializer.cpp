#include "RecordingSerializer.h"

#include "Util/IRecordingSlice.h"

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
						trajs << ", ";
					}

					trajs << info.posX << ", " << info.posY;
					first = false;
				}
				trajs << std::endl;
			}
		}
	}
}