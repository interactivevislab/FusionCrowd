#pragma once

#include "FcServerApi.h"
#include "Export/Export.h"
#include "FCServerCore.h"
#include "RequestProcessor.h"
#include "MessageCodes.h"

#include <memory>
#include <map>


namespace FusionCrowdWeb {

	using namespace FusionCrowd;

	class FC_SERVER_API FusionCrowdServer
	{
	public:
		FusionCrowdServer();
		~FusionCrowdServer();

		void StartOn(const char* inIpAdress, short inPort);
		void Shutdown();

	private:
		FCServerCore _serverCore;

		std::shared_ptr<ISimulatorBuilder> _builder;
		std::shared_ptr<ISimulatorFacade> _simulator;
		bool _isSimulationStarted = false;

		std::map<RequestCode, IRequestProcessor*> _requestProcessors
		{
			{
				DoStep,
				new RequestProcessor<void, float>(&ISimulatorFacade::DoStep)
			},
			{
				SetAgentOp,
				new RequestProcessor<OperationStatus, size_t, ComponentId>(&ISimulatorFacade::SetAgentOp)
			},
			{
				SetAgentStrategy,
				new RequestProcessor<OperationStatus, size_t, ComponentId>(&ISimulatorFacade::SetAgentStrategy)
			},
			{
				SetAgentGoal,
				new RequestProcessor<OperationStatus, size_t, Point>(&ISimulatorFacade::SetAgentGoal)
			},
			{
				GetAgentCount,
				new RequestProcessor<size_t>(&ISimulatorFacade::GetAgentCount)
			},
			//{
			//	GetAgents,
			//	new RequestProcessor<>(&ISimulatorFacade::GetAgents)
			//},
			{
				AddAgent,
				new RequestProcessor<size_t, float, float, ComponentId, ComponentId, ComponentId>(&ISimulatorFacade::AddAgent)
			},
			{
				RemoveAgent,
				new RequestProcessor<OperationStatus, size_t>(&ISimulatorFacade::RemoveAgent)
			}
		};

		void InitBuilderByNavMeshPath(const char* inNavMeshPath);
		void InitBuilderByNavMeshName(const char* inNavMeshName);
		void StartSimulation();
		void ProcessRequest();
		void SendResponce(ResponseCode inResponseCode);
		void SendResponce(ResponseCode inResponseCode, const char * inResponseData, size_t inDataSize);
	};

}