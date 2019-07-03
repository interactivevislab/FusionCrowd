#pragma once

#include "Config.h"
#include "Util/IRecording.h"

#include <memory>

namespace TestFuctionCrowd
{
	// Track recording holder from Edinburgh Informatics Forum Pedestrian Database
	// http://homepages.inf.ed.ac.uk/rbf/FORUMTRACKING/

	class EIFPDRecording : public FusionCrowd::IRecording
	{
	public:
		//static EIFPDRecording Read(char* filepath);

		EIFPDRecording();

		FusionCrowd::TimeSpan GetTimeSpan() const override;
		const FusionCrowd::IRecordingSlice & GetSlice(float time) const override;

		~EIFPDRecording();
	private:
		//class Impl;

		//std::unique_ptr<Impl> pimpl;
	};
}
