#pragma once

#include "FcWebApi.h"


namespace FusionCrowdWeb
{
	//navmesh, full agents init data
	struct FC_WEB_API InitComputingData
	{
		float StubData;

		static void Serialize(const InitComputingData& inData, char* inDest);
		static InitComputingData Deserialize(const char* inRawData);
	};


	//lists of received, lost and secondary agents
	struct FC_WEB_API InputComputingData
	{
		float StubData;

		static void Serialize(const InputComputingData& inData, char* inDest);
		static InputComputingData Deserialize(const char* inRawData);
	};


	//agents data after computing iteration
	struct FC_WEB_API OutputComputingData
	{
		float StubData;

		static void Serialize(const OutputComputingData& inData, char* inDest);
		static OutputComputingData Deserialize(const char* inRawData);
	};
}
