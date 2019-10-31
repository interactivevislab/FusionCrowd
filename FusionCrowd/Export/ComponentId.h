#pragma once

#include "Export/Config.h"
#include <vector>


namespace FusionCrowd
{
	FUSION_CROWD_API typedef int ComponentId;

	// Unfortunately, theese ids can't be put into corresponding component's header files as they have to be accessible from outside of the library.
	//
	// Ids to be used in internal components:
	//  Operation components are 1xx
	//  Tactic    components are 2xx
	//  Strategy  components are 3xx
	//
	// For external components:
	//  Operation components are 7xx
	//  Tactic    components are 8xx
	//  Strategy  components are 9xx
	namespace ComponentIds
	{
		FUSION_CROWD_API const ComponentId KARAMOUZAS_ID = 100;
		FUSION_CROWD_API const ComponentId HELBING_ID    = 101;
		FUSION_CROWD_API const ComponentId ORCA_ID       = 102;
		FUSION_CROWD_API const ComponentId ZANLUNGO_ID   = 103;
		FUSION_CROWD_API const ComponentId PEDVO_ID      = 104;
		FUSION_CROWD_API const ComponentId SWITCHING     = 105;
		FUSION_CROWD_API const ComponentId GCF_ID        = 106;


		FUSION_CROWD_API const ComponentId NAVMESH_ID = 200;

		FUSION_CROWD_API const ComponentId FSM_ID        = 300;

		constexpr ComponentId allOperationComponentTypes[6] = {
			KARAMOUZAS_ID, HELBING_ID, ORCA_ID, ZANLUNGO_ID, PEDVO_ID, SWITCHING
		};
	}
}