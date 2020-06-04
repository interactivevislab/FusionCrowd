#include "RecordingSerializer.h"

#include "Export/IRecordingSlice.h"

#include <fstream>
#include <string>

namespace FusionCrowd
{
	namespace Recordings
	{
		namespace
		{
			const char SEP = ',';
		}

		void Serialize(IRecording const &  rec, char const * destFilePath, size_t pathLen)
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

				trajs << slice.GetTime();
				for(size_t id : ids)
				{
					auto info = slice.GetAgentInfo(id);
					trajs << SEP << id << SEP
					      << info.posX << SEP << info.posY << SEP
					      << info.orientX << SEP << info.orientY << SEP
					      << info.radius;
				}
				trajs << std::endl;
			}
		}
	}
}