#include "UE4StrategyProxy.h"

#include "Simulator.h"
#include "StrategyComponent/Goal/PointGoal.h"


UE4StrategyProxy::UE4StrategyProxy(FusionCrowd::ISimulatorFacade * simulator) : _simulator(simulator)
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


void UE4StrategyProxy::Update(float timeStep)
{
}

UE4StrategyProxy::~UE4StrategyProxy()
{
}
