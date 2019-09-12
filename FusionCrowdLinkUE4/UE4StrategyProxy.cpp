#include "UE4StrategyProxy.h"

UE4StrategyProxy::UE4StrategyProxy(FusionCrowd::ISimulatorFacade * simulator, FusionCrowd::ComponentId assignedId) : _simulator(simulator), _id(assignedId)
{
}

void UE4StrategyProxy::AddAgent(size_t id)
{
	_agents.insert(id);
}

bool UE4StrategyProxy::RemoveAgent(size_t id)
{
	return _agents.erase(id) > 0;
}

void UE4StrategyProxy::SetGoal(size_t id, const float* goalPos)
{
	if(_agents.find(id) != _agents.end())
	{
		_simulator->SetAgentGoal(id, goalPos[0], goalPos[1]);
	}
}

FusionCrowd::ComponentId UE4StrategyProxy::GetId()
{
	return _id;
}


void UE4StrategyProxy::Update(float timeStep)
{
}

UE4StrategyProxy::~UE4StrategyProxy()
{
}
