#pragma once

#include "Export/Config.h"
#include "Export/Export.h"
#include "Export/FCArray.h"

namespace FusionCrowd
{
	namespace Fsm
	{
		extern "C"
		{
			using State = size_t;
			using Event = size_t;

			class FUSION_CROWD_API IFsm
			{
			public:
				virtual State GetInitialState() const = 0;
				virtual State GetFinalState() const = 0;
				virtual State Advance(State currentState, Event input) = 0;
			};

			class FUSION_CROWD_API IFsmTransition
			{
			public:
				virtual Event When() const = 0;
				virtual State GetNext() = 0;
				virtual ~IFsmTransition() { };
			};

			class FUSION_CROWD_API IBuilderAddTransitions
			{
			public:
				virtual IBuilderAddTransitions* Add(State from, State to, Event when) = 0;
				virtual IBuilderAddTransitions* AddRandom(State from, FCArray<State> destOptions, Event when) = 0;

				virtual IFsm* Build() = 0;
			};

			class FUSION_CROWD_API IBuilderAddStates
			{
			public:
				virtual IBuilderAddStates* Initial(State state) = 0;
				virtual IBuilderAddStates* Final(State state) = 0;
				virtual IBuilderAddStates* Intermediate(State state) = 0;

				virtual IBuilderAddTransitions* WithTransitions() = 0;
			};

			class FUSION_CROWD_API IBuilder
			{
			public:
				virtual IBuilderAddStates* WithStates() = 0;
			};

			FUSION_CROWD_API void Deleter(IFsm* machine);
			FUSION_CROWD_API IBuilder* Builder();
			FUSION_CROWD_API void BuilderDeleter(IBuilder * builder);
		}
	}
}