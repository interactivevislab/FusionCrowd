#include "FsmStrategy.h"


namespace FusionCrowd
{
	namespace Strategy
	{
		FsmStrategy::FsmStrategy()
		{
		}

		FsmStrategy::~FsmStrategy()
		{
		}

		bool FsmStrategy::BuildFSM()
		{
			for (int i = 0; i < _agents.size(); i++)
			{
				bool isCorrectly = false;
				for (int j = 0; j < _states.size(); j++)
				{
					if (_states[j]->GetName() == _agents[i]._startStateName)
					{
						_agents[i]._state = _states[j];
						isCorrectly = true;
					}
				}
				if (!isCorrectly)
				{
					return false;
				}
			}
			for (int i =0; i < _transitions.size(); i++)
			{
				bool isCorrectlyFrom = false;
				bool isCorrectlyTo = false;
				for (int j = 0; j < _states.size(); j++)
				{
					if (_states[j]->GetName() == _transitions[i]->GetFrom())
					{
						_states[j]->AddTransition(_transitions[i]);
						isCorrectlyFrom = true;
					}
					if (_states[j]->GetName() == _transitions[i]->GetTo())
					{
						_transitions[i]->SetNextState(_states[j]);
						isCorrectlyTo = true;
					}
				}
				if ((!isCorrectlyFrom)||(!isCorrectlyTo))
				{
					return false;
				}
			}

			return true;
		}

		void FsmStrategy::Update()
		{
			for (int i = 0; i < _agents.size(); i++)
			{
				FSM::IFsmState * newState = NULL;
				_agents[i]._state->Update(_agents[i]._idAgent, newState);
				if (newState != NULL)
				{
					_agents[i]._state = newState;
				}
			}
		}

		Goal* FsmStrategy::GetGoal(int idAgent)
		{
			return _agents[idAgent]._state->GetGoal();
		}

		void FsmStrategy::AddState(FSM::IFsmState* state)
		{
			_states.push_back(state);
		}

		void FsmStrategy::AddTransitions(FSM::IFsmTransition* transition)
		{
			_transitions.push_back(transition);
		}
		void FsmStrategy::AddAgent(Agent* agent, std::string stateName)
		{
			_agents.push_back(FSM::FsmAgentInfo(agent, stateName, NULL));
		}
	}
}
