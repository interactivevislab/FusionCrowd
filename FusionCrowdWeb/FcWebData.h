#pragma once

#include "FcWebApi.h"
#include "Export/Export.h"
#include "FcFileWrapper.h"

#include <vector>


namespace FusionCrowdWeb
{
	using FusionCrowd::FCArray;


	struct FC_WEB_API AgentInitData
	{
		float X, Y;
		float GoalX, GoalY;
	};


	struct FC_WEB_API NavMeshRegion
	{
		float CenterX, CenterY;
		float Width = -1, Height = -1;

		NavMeshRegion();
		NavMeshRegion(const std::string& inNavMeshPath);
		bool IsPointInside(float inX, float inY);
		bool IsPointInsideBoundaryZone(float inX, float inY, float inBoundaryZoneDepth);
		void Split(size_t inNumParts, std::vector<NavMeshRegion>& outParts);
		std::vector<NavMeshRegion> Split(size_t inNumParts);
		bool IsValid();
	};


	//navmesh, full agents init data
	struct FC_WEB_API InitComputingData
	{
		InitComputingData();
		InitComputingData(const std::string& inNavMeshFileName, FCArray<AgentInitData> inAgentsData);

		FcFileWrapper NavMeshFile;
		FCArray<AgentInitData> AgentsData;

		NavMeshRegion NavMeshRegion;
	};


	//lists of received, lost and secondary agents
	struct FC_WEB_API InputComputingData
	{
		InputComputingData();
		InputComputingData(float inTimeStep);

		float TimeStep;

		FCArray<AgentInitData> NewAgents;
		FCArray<AgentInitData> BoundaryAgents;
	};


	//agents data after computing iteration
	struct FC_WEB_API OutputComputingData
	{
		OutputComputingData();
		OutputComputingData(FCArray<FusionCrowd::AgentInfo> inAgentInfos);

		FCArray<FusionCrowd::AgentInfo> AgentInfos;

		FCArray<FusionCrowd::AgentInfo> DisplacedAgents;
	};
}
