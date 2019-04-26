#pragma once

#include "Config.h"
#include "Agent.h"

class FUSION_CROWD_API IOperationComponent
{
public:
	IOperationComponent() {}
	virtual void Update(FusionCrowd::Agent* agent, float timeStep) {}
	virtual ~IOperationComponent() {}
};