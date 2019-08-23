#pragma once
#include "IFsmCondition.h"
#include "../../Agent.h"
#include "../../Config.h"

namespace FusionCrowd
{
	namespace Strategy
	{
		namespace FSM
		{
			class FUSION_CROWD_API ConditionGoal:
				public IFsmCondition
			{
			public:
				ConditionGoal();
				ConditionGoal(float distSq);
				~ConditionGoal();

				void SetDestSq(float distSq);
				bool ConditionMet(Agent* agent, const Goal* goal);


			private:
				float _distSq;
			};
		}
	}
}
