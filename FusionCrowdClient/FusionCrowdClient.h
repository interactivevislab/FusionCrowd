#pragma once

#include "FcClientApi.h"
#include "WebClientCore.h"
//#include "RequestProcessor.h"
#include "MessageCodes.h"


namespace FusionCrowdWeb
{
	//using namespace FusionCrowd;

	class FC_CLIENT_API FusionCrowdClient
	{
	public:
		void StartOn(const char* inIpAdress, short inPort);
		void Shutdown();

	private:
		WebClientCore _clientCore;

		//void InitBuilderByNavMeshPath(const char* inNavMeshPath);
		//void InitBuilderByNavMeshName(const char* inNavMeshName);
		//void StartSimulation();
		//void ProcessRequest();
		//void SendResponce(ResponseCode inResponseCode);
		//void SendResponce(ResponseCode inResponseCode, const char * inResponseData, size_t inDataSize);
	};
}