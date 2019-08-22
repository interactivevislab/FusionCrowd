#pragma once

#include <string>

#include "Navigation/NavSystem.h"

#include "TestCases/EIFPD/EIFPDRecording.h"
#include "TestCases/EIFPD/EIFPDStrategy.h"
#include "TestCases/EIFPD/EIFPDConfig.h"

namespace TestFusionCrowd
{
	class EIFPDDataset
	{
	public:
		EIFPDDataset(std::string path);

		EIFPDRecording GetRecording();
		EIFPDStrategy GetStrategy();
		EIFPDConfig GetConfig();
		FusionCrowd::NavSystem GetNavSystem();

		~EIFPDDataset();
	};
}
