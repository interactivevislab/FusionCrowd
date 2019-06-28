#pragma once

#include "Config.h"
#include "Util/FCArray.h"
#include "Util/IRecordingSlice.h"

namespace FusionCrowd
{
	using TimeSpan = FCArray<float>;

	class FUSION_CROWD_API IRecording
	{
	public:
		virtual TimeSpan GetTimeSpan() const = 0;
		virtual const IRecordingSlice & GetSlice(float time) = 0;

		virtual ~IRecording() { }
	};
}
