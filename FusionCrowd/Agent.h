#pragma once

#include "Config.h"

#include "OperationComponent/IOperationComponent.h"
#include "StrategyComponent/Goal/Goal.h"
#include "StrategyComponent/IStrategyComponent.h"

#include <memory>

namespace FusionCrowd
{
	class FUSION_CROWD_API Agent
	{
	public:
		Agent(size_t id);
		~Agent();

		size_t id;

		std::shared_ptr<IOperationComponent> opComponent;
		std::shared_ptr<IStrategyComponent> stratComponent;
		std::shared_ptr<Goal> currentGoal;
	};
}