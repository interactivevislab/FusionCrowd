#pragma once

#include "IFsm.h"
#include "Export/Config.h"
#include "Export/Export.h"

namespace FusionCrowd
{
	namespace Fsm
	{
		extern "C"
		{
			class FUSION_CROWD_API IStrategyConfigurator
			{
			public:
				virtual size_t AddMachine(IFsm* machine) = 0;

				virtual void CreateGoToAction(State duringState, float goalX, float goalY) = 0;

				virtual void SetTickEvent(Event fireEvt) = 0;
				virtual void CreateCloseToEvent(Event fireEvt, float pointX, float pointY) = 0;
			};

			struct FUSION_CROWD_API AgentParams : public ModelAgentParams
			{
				size_t FsmId;
			};
		}
	}
}
