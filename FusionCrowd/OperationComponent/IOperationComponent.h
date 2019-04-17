#pragma once

#include "Config.h"
#include "Agent.h"

class FUSION_CROWD_API IOperationComponent
{
public:
	virtual void Update(FusionCrowd::Agent* agent, float timeStep) {}
	virtual ~IOperationComponent() {}
};