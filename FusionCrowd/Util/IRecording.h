#pragma once

#include "Config.h"
#include "Util/FCArray.h"
#include "Util/IRecordingSlice.h"

namespace FusionCrowd
{
	extern "C"
	{
		using TimeSpan = FCArray<float>;

		class FUSION_CROWD_API IRecording
		{
		public:
			virtual size_t GetSlicesCount() const = 0;
			virtual void GetTimeSpan(TimeSpan & outTimeSpan) const = 0;
			virtual const IRecordingSlice & GetSlice(float time) const = 0;

			virtual ~IRecording() { }
		};
	}
}
