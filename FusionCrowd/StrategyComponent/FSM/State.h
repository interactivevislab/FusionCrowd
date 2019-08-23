#pragma once
#include "IFsmState.h"
#include "../Goal/Goal.h"
#include "../../Config.h"

#include <string>

namespace FusionCrowd
{
	namespace Strategy
	{
		namespace FSM
		{
			class FUSION_CROWD_API State :
				public IFsmState
			{
			public:
				State();
				State(std::string name, bool isFinal, Goal* goal);
				~State();

				void Update(Agent* agent, IFsmState* newState);
			private:

			};
		}
	}
}
