#pragma once

#include "OperationComponent/IOperationComponent.h"
#include "TacticComponent/ITacticComponent.h"
#include "Export/IStrategyComponent.h"

#include <memory>

namespace FusionCrowd
{
	class Goal;

	class Agent
	{
	public:
		Agent(size_t id);

		~Agent();

		size_t id;

		std::weak_ptr<IOperationComponent> opComponent;
		std::weak_ptr<ITacticComponent> tacticComponent;
		std::weak_ptr<IStrategyComponent> stratComponent;
		std::shared_ptr<Goal> currentGoal;
	};
}