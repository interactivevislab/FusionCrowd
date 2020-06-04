#pragma once

#include "FcComputationalServerApi.h"
#include "Export/Export.h"
#include "WebNode.h"
#include "WebMessage.h"

#include <memory>


namespace FusionCrowdWeb
{
	using namespace FusionCrowd;

	class FC_COMPUTATIONAL_SERVER_API FcComputationalServer : public WebNode
	{
	public:
		void StartServer(WebAddress inAddress) override;

		void AcceptMainServerConnection();
		void InitComputation();
		void ProcessComputationRequest();

	private:
		int _mainServerId;

		std::shared_ptr<ISimulatorBuilder> _builder;
		std::shared_ptr<ISimulatorFacade> _simulator;
	};
}
