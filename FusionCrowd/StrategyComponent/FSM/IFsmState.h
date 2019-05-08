#pragma once
#include "../Goal/Goal.h"
#include "../../Config.h"
#include "../../Agent.h"

#include <vector>
#include <string>

namespace FusionCrowd
{
	namespace Strategy
	{
		namespace FSM
		{
			// forward declarations
			class IFsmTransition;

			class FUSION_CROWD_API IFsmState
			{
			public:
				IFsmState() {};
				~IFsmState() {};

				virtual void Update(Agent* agent,IFsmState* newState) {};

				virtual Goal* GetGoal()
				{
					return _goal;
				}

				std::string GetName()
				{
					return _name;
				}

				void AddTransition(IFsmTransition* transition)
				{
					_transitions.push_back(transition);
				}

			protected:
				std::string _name;
				bool _isFinal;
				Goal* _goal;
				std::vector<IFsmTransition*> _transitions;
			};
		}
	}
}
