#pragma once

#include "FcComputationalServerApi.h"
#include "Export/Export.h"
#include "WebNode.h"


namespace FusionCrowdWeb
{
	class FC_COMPUTATIONAL_SERVER_API FcComputationalServer : public WebNode
	{
	public:
		void AcceptMainServerConnection();
		void InitComputation();
		void ProcessComputationRequest();

	private:
		int _mainServerId;

		std::shared_ptr<FusionCrowd::ISimulatorBuilder> _builder;
		std::shared_ptr<FusionCrowd::ISimulatorFacade> _simulator;

		NavMeshRegion _navMeshRegion;
	};
}
