#pragma once
#include "IFsmTransition.h"
#include "IFsmCondition.h"
#include "../../Config.h"

namespace FusionCrowd
{
	namespace Strategy
	{
		namespace FSM
		{
			class FUSION_CROWD_API Transition:
				public IFsmTransition
			{
			public:
				Transition();
				Transition(std::string from, std::string to, IFsmCondition* condition);
				Transition(IFsmState* state, IFsmCondition* condition);
				~Transition();

				void SetCondition(IFsmCondition* condition);
				//void SetNextState(IFsmState* state);

				bool CheckTransition(Agent* agent, const Goal* goal);

			private:
				IFsmCondition* _condition;
			};
		}
	}
}
