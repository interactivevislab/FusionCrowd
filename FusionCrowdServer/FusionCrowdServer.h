#pragma once

#include <memory>
#include <map>

#include "Export/Export.h"
#include "FCServerCore.h"
#include "RequestProcessor.h"

#include "MessageCodes.h"


namespace FusionCrowdWeb {

	using namespace FusionCrowd;

	class FusionCrowdServer
	{
	private:
		FCServerCore _serverCore;
		std::shared_ptr<ISimulatorBuilder> _builder;
		std::shared_ptr<ISimulatorFacade> _simulator;
		bool _isSimulationStarted = false;
		std::map<RequestCode, IRequestProcessor*> _requestProcessors {
			{ DoStep,			new RequestProcessor<void, float>										(&ISimulatorFacade::DoStep) },
			{ SetAgentOp,		new RequestProcessor<OperationStatus, size_t, ComponentId>				(&ISimulatorFacade::SetAgentOp) },
			{ SetAgentStrategy,	new RequestProcessor<OperationStatus, size_t, ComponentId>				(&ISimulatorFacade::SetAgentStrategy) },
			{ SetAgentGoal,		new RequestProcessor<OperationStatus, size_t, float, float>				(&ISimulatorFacade::SetAgentGoal) },
			{ GetAgentCount,	new RequestProcessor<size_t>											(&ISimulatorFacade::GetAgentCount) },
			//{ GetAgents,		new RequestProcessor<>													(&ISimulatorFacade::GetAgents) },
			{ AddAgent,			new RequestProcessor<size_t, float, float, ComponentId, ComponentId>	(&ISimulatorFacade::AddAgent) },
			{ RemoveAgent,		new RequestProcessor<OperationStatus, size_t>							(&ISimulatorFacade::RemoveAgent) }
		};

		void InitBuilderByNavMeshPath(const char* navMeshPath);
		void InitBuilderByNavMeshName(const char* navMeshName);
		void StartSimulation();
		void ProcessRequest();
		void SendResponce(ResponseCode responseCode);
		void SendResponce(ResponseCode responseCode, const char * responseData, size_t dataSize);

	public:
		FusionCrowdServer();
		~FusionCrowdServer();

		void StartOn(const char* ipAdress, short port);
		void Shutdown();
	};

}