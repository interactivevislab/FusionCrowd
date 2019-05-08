#pragma once
#include "../../Agent.h"
#include "../Goal/Goal.h"
#include "../../Config.h"

#include <string>

namespace FusionCrowd
{
	namespace Strategy
	{
		namespace FSM
		{
			// forward declarations
			class IFsmState;

			class FUSION_CROWD_API IFsmTransition
			{
			public:

				virtual bool CheckTransition(Agent* agent, const Goal* goal)
				{
					return false;
				};

				IFsmState* GetNextState()
				{
					return _nextState;
				}

				void SetNextState(IFsmState* nextState)
				{
					_nextState = nextState;
				}

				std::string GetFrom()
				{
					return _from;
				}

				std::string GetTo()
				{
					return _to;
				}

				virtual ~IFsmTransition() {};

			protected:
				IFsmState* _nextState;
				std::string _from;
				std::string _to;
			};
		}
	}
}

