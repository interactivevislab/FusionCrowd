#pragma once

#include "Config.h"
#include "Util/TimeSpan.h"
#include "Util/IRecordingSlice.h"

namespace FusionCrowd
{
	class FUSION_CROWD_API IRecording
	{
	public:
		virtual TimeSpan GetTimeSpan() const = 0;
		virtual const IRecordingSlice & GetSlice(float time) = 0;

		virtual ~IRecording() { }
	};
}
