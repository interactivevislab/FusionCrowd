#pragma once

#include "FcWebApi.h"
#include "Export/Export.h"


namespace FusionCrowdWeb
{
	struct FC_WEB_API AgentInitData
	{
		float X;
		float Y;
		float GoalX;
		float GoalY;
	};

	//navmesh, full agents init data
	struct FC_WEB_API InitComputingData
	{
		FusionCrowd::FCArray<AgentInitData> AgentsData;

		static size_t Serialize(const InitComputingData& inData, char*& outRawData);
		static InitComputingData Deserialize(const char* inRawData);
	};


	//lists of received, lost and secondary agents
	struct FC_WEB_API InputComputingData
	{
		float TimeStep;

		static size_t Serialize(const InputComputingData& inData, char*& outRawData);
		static InputComputingData Deserialize(const char* inRawData);
	};


	//agents data after computing iteration
	struct FC_WEB_API OutputComputingData
	{
		FusionCrowd::FCArray<FusionCrowd::AgentInfo> AgentInfos;

		static size_t Serialize(const OutputComputingData& inData, char*& outRawData);
		static OutputComputingData Deserialize(const char* inRawData);
	};

	namespace WebDataHelper
	{
		template<typename T>
		void WriteData(const T& inData, char*& outMemoryIterator)
		{
			size_t size = sizeof(T);
			std::memcpy(outMemoryIterator, &inData, size);
			outMemoryIterator += size;
		}


		template<typename T>
		T ReadData(const char*& outMemoryIterator)
		{
			T data = *reinterpret_cast<const T*>(outMemoryIterator);
			outMemoryIterator += sizeof(T);
			return data;
		}
	}
}
