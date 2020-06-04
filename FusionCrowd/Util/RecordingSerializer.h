#pragma once

#include "Export/IRecording.h"
#include "Export/FCArray.h"
#include "Export/Config.h"

namespace FusionCrowd
{
	namespace Recordings
	{
		void FUSION_CROWD_API Serialize(IRecording const &  rec, char const * destFilePath, size_t pathLen);
	}
}
