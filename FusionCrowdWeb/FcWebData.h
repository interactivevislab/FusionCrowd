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
	public:
		float CenterX, CenterY;
		float Width = -1, Height = -1;

		NavMeshRegion() = default;
		NavMeshRegion(const std::string& inNavMeshPath);

		bool IsPointInside(float inX, float inY);
		bool IsPointInsideBoundaryZone(float inX, float inY, float inBoundaryZoneDepth);

		std::vector<NavMeshRegion> Split(size_t inNumParts);

	private:
		void Split(size_t inNumParts, std::vector<NavMeshRegion>& outParts);
	};


	//navmesh, full agents init data
	struct FC_WEB_API InitComputingData
	{
		InitComputingData();
		InitComputingData(const std::string& inNavMeshFileName, FusionCrowd::FCArray<AgentInitData> inAgentsData);

		FcFileWrapper NavMeshFile;
		FusionCrowd::FCArray<AgentInitData> AgentsData;

		NavMeshRegion NavMeshRegion;
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

		FusionCrowd::FCArray<FusionCrowd::AgentInfo> DisplacedAgents;
	};


	//list of agents ids
	struct FC_WEB_API AgentsIds
	{
		AgentsIds();
		AgentsIds(size_t inNum);

		FusionCrowd::FCArray<size_t> Values;
	};
}
