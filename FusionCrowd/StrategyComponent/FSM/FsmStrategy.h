#pragma once
#include "../IStrategyComponent.h"
#include "../Goal/Goal.h"
#include "../../Config.h"
#include "Agent.h"

#include "IFsmState.h"
#include "IFsmTransition.h"

#include <vector>
#include <map>

namespace FusionCrowd
{
	namespace Strategy
	{
		namespace FSM
		{
			struct FUSION_CROWD_API FsmAgentInfo
			{
				Agent* _idAgent;
				std::string _startStateName;
				IFsmState* _state;
				FsmAgentInfo() {};
				FsmAgentInfo(Agent* idAgent, std::string startStateName, IFsmState* state):
					_idAgent(idAgent), _startStateName(startStateName), _state(state)
				{}
			};
		}

		class FUSION_CROWD_API FsmStrategy:
			public IStrategyComponent
		{
		public:
			FsmStrategy();
			~FsmStrategy();

			bool BuildFSM();
			void Update();

			Goal* GetGoal(int idAgent);

			void AddState(FSM::IFsmState* state);
			void AddTransitions(FSM::IFsmTransition* transition);
			void AddAgent(Agent* agent, std::string stateName);
		private:
			std::vector<FSM::IFsmState*> _states;
			std::vector<FSM::IFsmTransition*> _transitions;
			std::vector<FSM::FsmAgentInfo> _agents;

		};
	}
}
