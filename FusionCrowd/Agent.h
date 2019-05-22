#pragma once

#include "Config.h"

#include "TacticComponent/Path/PrefVelocity.h"

#include <vector>

using namespace DirectX::SimpleMath;

namespace FusionCrowd
{
	class Goal;

	class FUSION_CROWD_API Agent
	{
	public:
		Agent(size_t id, Goal & goal);
		~Agent();

		size_t id;

		Goal & getCurrentGoal() const;

	private:
		std::reference_wrapper<Goal> _currentGoal;
	};
}