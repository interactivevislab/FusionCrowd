#pragma once

#include "FusionCrowdWebApi.h"


namespace FusionCrowdWeb
{
	enum FC_WEB_API RequestCode : unsigned char
	{
		StartWithNavMesh,	// void InitBuilderByNavMeshName(const char* navMeshName) + void StartSimulation()
		DoStep,				// void DoStep(float timeStep = 0.1f)
		SetAgentOp,			// OperationStatus SetAgentOp(size_t agentId, ComponentId opId)
		SetAgentStrategy,	// OperationStatus SetAgentStrategy(size_t agentId, ComponentId strategyId)
		SetAgentGoal,		// OperationStatus SetAgentGoal(size_t agentId, float x, float y)
		GetAgentCount,		// size_t GetAgentCount()
		GetAgents,			// bool GetAgents(FCArray<AgentInfo> & output)
		AddAgent,			// size_t AddAgent(float x, float y, ComponentId opId, ComponentId strategyId)
		RemoveAgent			// OperationStatus RemoveAgent(size_t agentId)
	};

	enum FC_WEB_API ResponseCode : unsigned char
	{
		Success,
		NeedRunSimulation,
		UnknowsRequestCode,
		InnerFusionCrowdError
	};

	struct FC_WEB_API WebMessageHead
	{
		union WebCode
		{
			RequestCode RequestCode;
			ResponseCode ResponseCode;
		} WebCode;
		size_t MessageLenght;
	};
}