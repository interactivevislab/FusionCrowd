#pragma once

#include "OperationComponent/IOperationComponent.h"
#include "TacticComponent/ITacticComponent.h"
#include "Export/IStrategyComponent.h"
#include "StrategyComponent/Goal/Goal.h"

#include <memory>

namespace FusionCrowd
{
	class Agent
	{
	public:
		Agent(size_t id, Goal goal);

		const size_t id;

		std::weak_ptr<IOperationComponent> opComponent;
		std::weak_ptr<ITacticComponent> tacticComponent;
		std::weak_ptr<IStrategyComponent> stratComponent;

		Goal currentGoal;

		size_t GetGroupId() const;
		void SetGroupId(size_t newGroupId);
	private:
		size_t _groupId;
	};
}