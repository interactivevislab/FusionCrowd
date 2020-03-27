#pragma once

#include "Export/Config.h"
#include "Export//FCArray.h"
#include "Export//IRecordingSlice.h"

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
			virtual const IRecordingSlice & GetCurrentSlice() const = 0;
			virtual const IRecordingSlice * Begin() const = 0;
			virtual const IRecordingSlice * End() const = 0;
			virtual bool LoadFromFile(char const * path, size_t path_length) = 0;
			virtual void Serialize(char const * destFilePath, size_t pathLen) const = 0;

			virtual ~IRecording() { }
		};
	}
}
