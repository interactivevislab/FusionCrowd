#pragma once

#include "Config.h"
#include "Goal/Goal.h"

#include <map>


class FUSION_CROWD_API IStrategyComponent
{
public:

	virtual void Update() {}
	virtual void AddGoal(int idAgent, Goal* goal){}
	virtual Goal* GetGoal(int idAgent) { return NULL; }
	virtual ~IStrategyComponent() {}
private:

};
