#pragma once

#include "FcWebApi.h"


namespace FusionCrowdWeb
{
	//navmesh, full agents init data
	struct FC_WEB_API InitComputingData
	{
		int StubData;
	};


	//lists of received, lost and secondary agents
	struct FC_WEB_API InputComputingData
	{
		int StubData;
	};


	//agents data after computing iteration
	struct FC_WEB_API OutputComputingData
	{
		int StubData;
	};
}
