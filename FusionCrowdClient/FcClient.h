#pragma once

#include "FcClientApi.h"
#include "WebNode.h"
//#include "WebMessage.h"


namespace FusionCrowdWeb
{
	//using namespace FusionCrowd;

	class FC_CLIENT_API FusionCrowdClient
	{
	public:
		void StartOn(const char* inIpAdress, short inPort);

	private:
		WebNode _webNode;

		//void InitBuilderByNavMeshPath(const char* inNavMeshPath);
		//void InitBuilderByNavMeshName(const char* inNavMeshName);
		//void StartSimulation();
		//void ProcessRequest();
		//void SendResponce(ResponseCode inResponseCode);
		//void SendResponce(ResponseCode inResponseCode, const char * inResponseData, size_t inDataSize);
	};
}