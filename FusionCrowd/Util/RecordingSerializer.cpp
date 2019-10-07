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

			auto timespan = rec.GetTimeSpan();
			for(float step : timespan)
			{
				auto & slice = rec.GetSlice(step);

				auto ids = slice.GetAgentIds();
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