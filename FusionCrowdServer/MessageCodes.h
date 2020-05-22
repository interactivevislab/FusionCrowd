#pragma once


namespace FusionCrowdWeb
{
	enum RequestCode : unsigned char
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

	enum ResponseCode : unsigned char
	{
		Success,
		NeedRunSimulation,
		UnknowsRequestCode,
		InnerFusionCrowdError
	};
}