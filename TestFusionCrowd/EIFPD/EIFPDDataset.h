#pragma once

#include "Config.h"
#include "EIFPD/EIFPDRecording.h"
#include "EIFPD/EIFPDStrategy.h"
#include "EIFPD/EIFPDConfig.h"

namespace TestFuctionCrowd
{
	class EIFPDDataset
	{
	public:
		EIFPDDataset(char* path);

		EIFPDRecording GetRecording();
		//EIFPDStrategy GetStrategy();
		EIFPDConfig GetConfig();

		~EIFPDDataset();
	};
}
