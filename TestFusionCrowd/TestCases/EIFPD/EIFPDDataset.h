#pragma once

#include <string>
#include <memory>

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
		std::shared_ptr<EIFPDStrategy> EIFPDDataset::GetStrategy();
		EIFPDConfig GetConfig();
		FusionCrowd::NavSystem GetNavSystem();
		std::string GetNavMeshPath();

		~EIFPDDataset();
	};
}
