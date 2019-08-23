#include "ConditionGoal.h"


namespace FusionCrowd
{
	namespace Strategy
	{
		namespace FSM
		{
			ConditionGoal::ConditionGoal(): IFsmCondition(), _distSq(0.f)
			{
			}


			ConditionGoal::ConditionGoal(float distSq) : IFsmCondition(), _distSq(distSq)
			{
			}

			ConditionGoal::~ConditionGoal()
			{
			}

			void ConditionGoal::SetDestSq(float distSq)
			{
				_distSq = distSq;
			}

			bool ConditionGoal::ConditionMet(Agent* agent, const Goal* goal)
			{
				float distSq = goal->squaredDistance(agent->_pos);
				return  distSq <= _distSq;
			}
		}
	}
}