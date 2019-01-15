#pragma once

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

	IStrategyComponent* strategyComponent;
	ITacticComponent* tacticComponent;
	IOperComponent* operComponent;

};

