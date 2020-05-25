#pragma once

#include "FcServerApi.h"
#include "Export/Export.h"
#include "WebServerCore.h"
#include "RequestProcessor.h"
#include "MessageCodes.h"

#include <memory>
#include <map>


namespace FusionCrowdWeb
{
	using namespace FusionCrowd;

	class FC_SERVER_API FusionCrowdServer
	{
	public:
		FusionCrowdServer();
		~FusionCrowdServer();

		void StartOn(const char* inIpAdress, short inPort);
		void Shutdown();

	private:
		WebServerCore _serverCore;

		std::shared_ptr<ISimulatorBuilder> _builder;
		std::shared_ptr<ISimulatorFacade> _simulator;
		bool _isSimulationStarted = false;

		std::map<RequestCode, IRequestProcessor*> _requestProcessors;

		void InitBuilderByNavMeshPath(const char* inNavMeshPath);
		void InitBuilderByNavMeshName(const char* inNavMeshName);
		void StartSimulation();
		void ProcessRequest();
		void SendResponce(ResponseCode inResponseCode);
		void SendResponce(ResponseCode inResponseCode, const char * inResponseData, size_t inDataSize);
	};
}
