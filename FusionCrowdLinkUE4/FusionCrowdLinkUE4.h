#pragma once

#include "Math/consts.h"
#include "Goal/GoalSet.h"
#include "Goal/Goal.h"
#include "Goal/PointGoal.h"
#include "Simulator.h"
#include "Agent.h"
#include "IOperComponent.h"
#include "OperationComponent/HelbingComponent.h"
#include "OperationComponent/SpatialQuery/NavMeshSpatialQuery.h"
#include "NavComponents/NavMesh/NavMeshLocalizer.h"
#include "NavComponents/NavMeshCompnent.h"


#define LINKFUSIONCROWD_API __declspec(dllexport)

struct agentInfo
{
	float* pos;
};

class FusionCrowdLinkUE4
{
public:
	LINKFUSIONCROWD_API FusionCrowdLinkUE4();
	LINKFUSIONCROWD_API ~FusionCrowdLinkUE4();

	LINKFUSIONCROWD_API void StartFusionCrowd();
	LINKFUSIONCROWD_API void DoStep();
	LINKFUSIONCROWD_API int GetAgentCount();
	LINKFUSIONCROWD_API void AddAgent(int agentsCount);
	LINKFUSIONCROWD_API void GetPositionAgents(agentInfo* agentsPos);

private:
	Simulator sim;
	int agentsCount;
};