#pragma once

#include <string>
#include <unordered_set>

#include "StrategyComponent/IStrategyComponent.h"
#include "Export.h"

class UE4StrategyProxy : public FusionCrowd::IStrategyComponent
{
public:
	UE4StrategyProxy(FusionCrowd::ISimulatorFacade * simulator, ComponentId assignedId);
	~UE4StrategyProxy();

	ComponentId GetId() override;

	void SetGoal(size_t id, const float * goalPos);

	void AddAgent(size_t id) override;
	bool RemoveAgent(size_t id) override;
	void Update(float timeStep) override;

private:
	FusionCrowd::ISimulatorFacade * _simulator;
	std::unordered_set<size_t> _agents;
};

