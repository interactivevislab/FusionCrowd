#pragma once

#include <vector>

#include "NavSystem.h"
#include "Agent.h"
#include "IStrategyComponent.h"
#include "ITacticComponent.h"
#include "IOperComponent.h"

class Simulator
{
public:
	Simulator();

	void DoStep();

	~Simulator();

private:
	NavSystem navSystem;
	std::vector<Agent> agents;
	std::vector<IStrategyComponent> strategyComponent;
	std::vector<ITacticComponent> tacticComponent;
	std::vector<IOperComponent> operComponent;
};

