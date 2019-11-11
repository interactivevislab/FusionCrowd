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

				virtual void CreateGoToAction(
					const size_t machineId,
					const State duringState,
					const Point goal
				) = 0;

				virtual void CreateGoToAnyAction(
					const size_t machineId,
					const State duringState,
					const FCArray<Point> & goals
				) = 0;

				virtual void CreatePointReachEvent(
					const size_t machineId,
					const Event fireEvt,
					const Point point,
					const float radius = .5f
				) = 0;

				virtual void CreateAnyPointReachEvent(
					const size_t machineId,
					const Event fireEvt,
					const FCArray<Point> & points,
					const float radius = .5f
				) = 0;

				virtual void CreateTimerEvent(
					const size_t machineId,
					const Fsm::State duringState,
					const Fsm::Event fireEvt,
					const float minWaitTime,
					const float maxWaitTime
				) = 0;

				virtual void SetTickEvent(
					const size_t machineId,
					const Event fireEvt
				) = 0;
			};

			struct FUSION_CROWD_API AgentParams : public ModelAgentParams
			{
				size_t FsmId;
			};
		}
	}
}
