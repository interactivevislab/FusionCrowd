#pragma once

#include "Config.h"

#include "OperationComponent/IOperationComponent.h"
#include "TacticComponent/Path/PrefVelocity.h"
#include "StrategyComponent/Goal/Goal.h"

#include <vector>
#include <memory>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	class FUSION_CROWD_API Agent
	{
	public:
		Agent(size_t id);
		~Agent();

		size_t id;

		std::shared_ptr<IOperationComponent> opComponent;
		std::shared_ptr<Goal> currentGoal;

	};
}