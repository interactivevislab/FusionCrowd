#pragma once

#include <map>
#include <vector>

#include "Config.h"

namespace FusionCrowd
{
	class Goal;

	class FUSION_CROWD_API GoalSet
	{
	public:
		GoalSet();
		~GoalSet();
		bool addGoal(size_t id, Goal * goal);
		Goal * getGoalByID(size_t id);
		Goal * getGoalByIDConcurrent(size_t id);
		Goal * getIthGoal(size_t i);
		Goal * getIthGoalConcurrent(size_t i);
		size_t size() const { return _goalIDs.size(); }
		size_t sizeConcurrent() const;
		Goal * getRandomGoal();
		Goal * getRandomWeightedGoal();

		friend class Goal;
	protected:
		void setGoalFull(const Goal * goal) const;
		void setGoalAvailable(const Goal * goal) const;
		std::map< size_t, Goal * >	_goals;
		mutable std::vector< size_t >	_goalIDs;
		mutable float	_totalWeight;

	};
}
