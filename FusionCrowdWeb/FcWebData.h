#pragma once

#include "FcWebApi.h"
#include "Export/Export.h"
#include "FcFileWrapper.h"


namespace FusionCrowdWeb
{
	struct FC_WEB_API AgentInitData
	{
		float X;
		float Y;
		float GoalX;
		float GoalY;
	};


	//navmesh, full agents init data
	struct FC_WEB_API InitComputingData
	{
		FcFileWrapper NavMeshFile;
		FusionCrowd::FCArray<AgentInitData> AgentsData;
	};


	//lists of received, lost and secondary agents
	struct FC_WEB_API InputComputingData
	{
		float TimeStep;
	};


	//agents data after computing iteration
	struct FC_WEB_API OutputComputingData
	{
		FusionCrowd::FCArray<FusionCrowd::AgentInfo> AgentInfos;
	};
}
