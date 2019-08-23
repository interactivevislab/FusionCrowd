#include "Transition.h"
#include <string>

namespace FusionCrowd
{
	namespace Strategy
	{
		namespace FSM
		{
			Transition::Transition()
			{
			}

			Transition::Transition(std::string from, std::string to, IFsmCondition* condition)
			{
				_from = from;
				_to = to;
				_condition = condition;
			}

			Transition::Transition(IFsmState* state, IFsmCondition* condition)
			{
				_condition = condition;
				_nextState = state;
			}

			Transition::~Transition()
			{
			}

			void Transition::SetCondition(IFsmCondition* condition)
			{
				_condition = condition;
			}

			/*void Transition::SetNextState(IFsmState* state)
			{
				_nextState = state;
			}*/

			bool  Transition::CheckTransition(Agent* agent, const Goal* goal)
			{
				return _condition->ConditionMet(agent, goal);
			}
		}
	}
}
