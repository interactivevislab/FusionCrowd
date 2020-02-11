#include "Export/Fsm/IFsm.h"

#include <map>
#include <random>
#include <memory>

namespace FusionCrowd
{
	namespace Fsm
	{
		using FsmTransitions = std::map<Event, std::unique_ptr<IFsmTransition>>;
		using FsmDict = std::map<State, FsmTransitions>;

		class SingleTransition : public IFsmTransition
		{
		public:
			SingleTransition(State target, Event when) : _target(target), _when(when) { }

			Event When() const override { return _when; }
			State GetNext() override { return _target; }
		private:
			Event _when;
			State _target;
		};

		class RandomTransition : public IFsmTransition
		{
		public:
			RandomTransition(FCArray<State> targets, Event when) :
				_targets(targets), _when(when), _random_engine((std::random_device())())
			{
			}

			Event When() const override { return _when; }
			State GetNext()
			{
				std::uniform_int_distribution<int> gen(0, _targets.size() - 1);

				return _targets[gen(_random_engine)];
			}
		private:
			Event _when;
			FCArray<State> _targets;
			std::mt19937 _random_engine;
		};

		class FsmImpl : public IFsm
		{
		public:
			FsmImpl(State initial, State final, FsmDict && transitions) : _initial(initial), _final(final), _transitions(std::move(transitions))
			{
			}

			State GetInitialState() const override { return _initial; }
			State GetFinalState() const override { return _final; }

			State Advance(State currentState, Event input) override
			{
				if(_transitions.find(currentState) == _transitions.end())
					return currentState;

				auto & transitionsMap = _transitions[currentState];

				if(transitionsMap.find(input) == transitionsMap.end())
					return currentState;

				return transitionsMap[input]->GetNext();
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
				auto result = new FsmImpl(_initial, _final, std::move(_dict));
				_dict.clear();

				return result;
			}

			IBuilderAddTransitions* Add(State from, State to, Event when) override
			{
				_dict[from].insert({ when, std::make_unique<SingleTransition>(to, when) });

				return this;
			}

			IBuilderAddTransitions* AddRandom(State from, FCArray<State> destOptions, Event when) override
			{
				_dict[from].insert({ when, std::make_unique<RandomTransition>(destOptions, when) });

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

		void FsmBuilderDeleter(IBuilder * builder)
		{
			//delete builder;
		}
	}
}
