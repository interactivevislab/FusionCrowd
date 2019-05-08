#include "State.h"
#include "IFsmTransition.h"


namespace FusionCrowd
{
	namespace Strategy
	{
		namespace FSM
		{
			State::State()
			{
			}

			State::State(std::string name, bool isFinal, Goal* goal)
			{
				_name = name;
				_isFinal = isFinal;
				_goal = goal;
			}


			State::~State()
			{
			}

			void State::Update(Agent* agent, IFsmState* newState)
			{
				if (!_isFinal)
				{
					for (int i = 0; i < _transitions.size(); i++)
					{
						bool res = _transitions[i]->CheckTransition(agent, _goal);
					}
				}
			}
		}
	}
}
