#pragma once

#include "Util/IRecording.h"
#include "Util/FCArray.h"
#include "Config.h"

namespace FusionCrowd
{
	namespace Recordings
	{
		void FUSION_CROWD_API Serialize(IRecording & rec, char const * destFilePath, size_t pathLen);
	}
}
