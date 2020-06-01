#pragma once

#include "FcComputationalServerApi.h"
#include "Export/Export.h"
#include "WebNode.h"
#include "RequestProcessor.h"
#include "WebMessage.h"

#include <memory>
#include <map>


namespace FusionCrowdWeb
{
	using namespace FusionCrowd;

	class FC_COMPUTATIONAL_SERVER_API FcComputationalServer : public WebNode
	{
	public:
		void StartServer(const char* inIpAdress, short inPort) override;

		void AcceptMainServerConnection();
		void InitComputation();
		void ProcessComputationRequest();

		//FcComputationalServer();
		//~FcComputationalServer();

		//void StartOn(const char* inIpAdress, short inPort);

	private:
		int _mainServerId;
		float _stubData;
		//std::shared_ptr<ISimulatorBuilder> _builder;
		//std::shared_ptr<ISimulatorFacade> _simulator;
		//bool _isSimulationStarted = false;

		//std::map<RequestCode, IRequestProcessor*> _requestProcessors;

		//void InitBuilderByNavMeshPath(const char* inNavMeshPath);
		//void InitBuilderByNavMeshName(const char* inNavMeshName);
		//void StartSimulation();
		//void ProcessRequest(int inClientId);
		//void Send(int inClientId, ResponseCode inResponseCode);
		//void Send(int inClientId, ResponseCode inResponseCode, const char * inResponseData, size_t inDataSize);
	};
}
