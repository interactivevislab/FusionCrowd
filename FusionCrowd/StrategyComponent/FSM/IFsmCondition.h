#pragma once
#include "../../Agent.h"
#include "../Goal/Goal.h"
#include "../../Config.h"

namespace FusionCrowd
{
	namespace Strategy
	{
		namespace FSM
		{
			class FUSION_CROWD_API IFsmCondition
			{
			public:
				IFsmCondition() {};
				~IFsmCondition() {};

				virtual bool ConditionMet(Agent* agent, const Goal* goal)
				{
					return false;
				}
			};
		}
	}
}