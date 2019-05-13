#include "GoalSet.h"
#include "Goal.h"

namespace FusionCrowd
{
	GoalSet::GoalSet() : _goals(), _goalIDs(),
	                     _totalWeight(0.f)
	{
	}

	GoalSet::~GoalSet()
	{
		std::map<size_t, Goal *>::iterator itr = _goals.begin();
		for (; itr != _goals.end(); ++itr)
		{
			itr->second->destroy();
		}
	}

	bool GoalSet::addGoal(size_t id, Goal* goal)
	{
		bool valid = false;
		if (_goals.find(id) == _goals.end())
		{
			valid = true;
			goal->_goalSet = this;
			_goals[id] = goal;
			_goalIDs.push_back(id);
			_totalWeight += goal->_weight;
		}
		return valid;
	}

	Goal* GoalSet::getGoalByID(size_t id)
	{
		Goal* goal = 0x0;
		std::map<size_t, Goal *>::const_iterator itr = _goals.find(id);
		if (itr != _goals.end() && itr->second->hasCapacity())
		{
			goal = itr->second;
		}
		return goal;
	}

	Goal* GoalSet::getGoalByIDConcurrent(size_t id)
	{
		Goal* goal = getGoalByID(id);
		return goal;
	}

	Goal* GoalSet::getIthGoal(size_t i)
	{
		Goal* goal = 0x0;
		if (i < _goalIDs.size())
		{
			size_t id = _goalIDs[i];
			std::map<size_t, Goal *>::const_iterator itr = _goals.find(id);
			if (itr != _goals.end() && itr->second->hasCapacity())
			{
				goal = itr->second;
			}
		}
		return goal;
	}

	Goal* GoalSet::getIthGoalConcurrent(size_t i)
	{
		Goal* goal = getIthGoal(i);
		return goal;
	}

	size_t GoalSet::sizeConcurrent() const
	{
		size_t s = _goalIDs.size();
		return s;
	}

	Goal* GoalSet::getRandomGoal()
	{
		Goal* goal = 0x0;
		const size_t GOAL_COUNT = _goalIDs.size();
		if (GOAL_COUNT > 0)
		{
			size_t idx = (size_t)(GOAL_COUNT * 1);
			idx = idx < GOAL_COUNT ? idx : GOAL_COUNT - 1;
			size_t id = _goalIDs[idx];
			std::map<size_t, Goal *>::const_iterator itr = _goals.find(id);
			goal = itr->second;
		}
		return goal;
	}

	Goal* GoalSet::getRandomWeightedGoal()
	{
		// TODO: Change this to use _goalIDs as the key interface of available goals
		Goal* tgtGoal = 0x0;
		if (_goalIDs.size() > 0)
		{
			const float TGT_WEIGHT = _totalWeight * 1;

			std::map<size_t, Goal *>::const_iterator itr = _goals.find(_goalIDs[0]);
			tgtGoal = itr->second;
			float accumWeight = tgtGoal->_weight;
			for (size_t i = 1; i < _goalIDs.size(); ++i)
			{
				if (accumWeight > TGT_WEIGHT) break;
				itr = _goals.find(_goalIDs[i]);
				tgtGoal = itr->second;
				accumWeight += tgtGoal->_weight;
			}
		}
		return tgtGoal;
	}

	void GoalSet::setGoalFull(const Goal* goal) const
	{
		size_t i = 0;
		std::map<size_t, Goal *>::const_iterator itr;
		while (i < _goalIDs.size())
		{
			itr = _goals.find(_goalIDs[i]);
			const Goal* testGoal = itr->second;
			if (testGoal == goal)
			{
				_totalWeight -= goal->_weight;
				_goalIDs.erase(_goalIDs.begin() + i); // todo: should this just be itr?
				break;
			}
			else
			{
				++i;
			}
		}
	}

	void GoalSet::setGoalAvailable(const Goal* goal) const
	{
		const size_t GOAL_ID = goal->getID();
		assert(_goals.find(GOAL_ID) != _goals.end() &&
			"Trying to set a goal available that doesn't belong to the goal set");
#ifdef _DEBUG
	bool found = false;
	for (size_t i = 0; i < _goalIDs.size(); ++i) {
		if (_goalIDs[i] == GOAL_ID) {
			found = true;
			break;
		}
	}
	assert(!found && "Trying to reactivate a goal that was never marked unavailable");
#endif
		_goalIDs.push_back(GOAL_ID);
		_totalWeight += goal->_weight;
	}
}
