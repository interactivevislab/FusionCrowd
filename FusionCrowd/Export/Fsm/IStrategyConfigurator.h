#pragma once

#include "IFsm.h"
#include "Export/Config.h"
#include "Export/Export.h"
#include "Export/FCArray.h"

namespace FusionCrowd
{
	namespace Fsm
	{
		extern "C"
		{
			struct FUSION_CROWD_API Point
			{
				float x;
				float y;
			};

			class FUSION_CROWD_API IStrategyConfigurator
			{
			public:
				virtual size_t AddMachine(IFsm* machine) = 0;

				virtual void CreateGoToAction(const State duringState, const Point goal) = 0;
				virtual void CreatePointReachEvent(const Event fireEvt, const Point point, const float radius = .5f) = 0;
				virtual void CreateAnyPointReachEvent(const Event fireEvt, const FCArray<Point> & points, const float radius = .5f) = 0;

				virtual void SetTickEvent(const Event fireEvt) = 0;
			};

			struct FUSION_CROWD_API AgentParams : public ModelAgentParams
			{
				size_t FsmId;
			};
		}
	}
}
