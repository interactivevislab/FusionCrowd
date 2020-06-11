#pragma once

#include "FcWebApi.h"
#include "Export/Export.h"
#include "FcFileWrapper.h"

#include <vector>


namespace FusionCrowdWeb
{
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
		InitComputingData(const std::string& inNavMeshFileName, FusionCrowd::FCArray<AgentInitData> inAgentsData);

		FcFileWrapper NavMeshFile;
		FusionCrowd::FCArray<AgentInitData> AgentsData;

		NavMeshRegion NavMeshRegion;
		FusionCrowd::FCArray<AgentInitData> BoundaryAgentsData;
	};


	//lists of received, lost and secondary agents
	struct FC_WEB_API InputComputingData
	{
		InputComputingData();
		InputComputingData(float inTimeStep);

		float TimeStep;

		FusionCrowd::FCArray<FusionCrowd::AgentInfo> NewAgents;
		FusionCrowd::FCArray<FusionCrowd::AgentInfo> BoundaryAgents;
	};


	//agents data after computing iteration
	struct FC_WEB_API OutputComputingData
	{
		OutputComputingData();
		OutputComputingData(FusionCrowd::FCArray<FusionCrowd::AgentInfo> inAgentInfos);

		FusionCrowd::FCArray<FusionCrowd::AgentInfo> AgentInfos;

		FusionCrowd::FCArray<FusionCrowd::AgentInfo> MissingAgents;
	};
}
