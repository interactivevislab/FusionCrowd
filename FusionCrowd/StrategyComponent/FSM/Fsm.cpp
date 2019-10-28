#include "Export/Fsm/IFsm.h"

#include <unordered_map>

namespace FusionCrowd
{
	namespace Fsm
	{
		using FsmTransitions = std::unordered_map<Event, State>;
		using FsmDict = std::unordered_map<State, FsmTransitions>;

		class FsmImpl : public IFsm
		{
		public:
			FsmImpl(State initial, State final, FsmDict transitions) : _transitions(transitions), _initial(initial), _final(final)
			{
			}

			State GetInitialState() const override { return _initial; }
			State GetFinalState() const override { return _final; }

			State Advance(State currentState, Event input) override
			{
				if(_transitions.find(currentState) == _transitions.end())
					return currentState;

				auto & transMap = _transitions.find(currentState)->second;

				if(transMap.find(input) == transMap.end())
					return currentState;

				return transMap[input];
			}

		private:
			State _initial;
			State _final;
			FsmDict _transitions;
		};

		class BuilderImpl : public IBuilder, IBuilderAddStates, IBuilderAddTransitions
		{
		public:
			IFsm* Build() override
			{
				auto result = new FsmImpl(_initial, _final, _dict);
				_dict.clear();

				return result;
			}

			IBuilderAddTransitions* Add(State from, State to, Event when) override
			{
				_dict[from].insert({when, to});

				return this;
			}

			IBuilderAddStates* Initial(State state) override
			{
				_initial = state;
				_dict.insert({state, FsmTransitions()});

				return this;
			}

			IBuilderAddStates* Final(State state) override
			{
				_final = state;
				_dict.insert({state, FsmTransitions()});

				return this;
			}

			IBuilderAddStates* Intermediate(State state) override
			{
				_dict.insert({state, FsmTransitions()});

				return this;
			}

			IBuilderAddTransitions* WithTransitions() override
			{
				return this;
			}

			IBuilderAddStates* WithStates() override
			{
				return this;
			}
		private:
			State _initial;
			State _final;
			FsmDict _dict;
		};

		void Deleter(IFsm* machine)
		{
			delete machine;
		}

		IBuilder* Builder()
		{
			return new BuilderImpl();
		}

		/*void BuilderDeleter(IBuilder * builder)
		{
			delete builder;
		}*/
	}
}
