#pragma once

#include <string>
#include <unordered_set>

#include "StrategyComponent/IStrategyComponent.h"


namespace FusionCrowd
{
	class Simulator;
}

class UE4StrategyProxy : public FusionCrowd::IStrategyComponent
{
public:
	UE4StrategyProxy(FusionCrowd::Simulator & simulator);
	~UE4StrategyProxy();

	std::string GetName() { return "ue4strategyproxy"; };
	void AddAgent(size_t id);
	void SetGoal(size_t id, const float * goalPos);
	bool RemoveAgent(size_t id);
	void Update(float timeStep);

private:
	FusionCrowd::Simulator & _simulator;
	std::unordered_set<size_t> _agents;
};

