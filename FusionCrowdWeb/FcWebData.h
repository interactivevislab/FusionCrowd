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
	};


	//lists of received, lost and secondary agents
	struct FC_WEB_API InputComputingData
	{
		float TimeStep;
	};


	//agents data after computing iteration
	struct FC_WEB_API OutputComputingData
	{
		FusionCrowd::FCArray<FusionCrowd::AgentInfo> AgentInfos;
	};


	template<typename T>
	class WebDataSerializer
	{
	public:
		static size_t Serialize(const T& inData, char*& outRawData)
		{
			static_assert(sizeof(T) == 0, "Unimplemented function");
		}

		static T Deserialize(const char* inRawData)
		{
			static_assert(sizeof(T) == 0, "Unimplemented function");
		}
	};


	template<>
	class FC_WEB_API WebDataSerializer<InitComputingData>
	{
	public:
		static size_t Serialize(const InitComputingData& inData, char*& outRawData);
		static InitComputingData Deserialize(const char* inRawData);
	};


	template<>
	class FC_WEB_API WebDataSerializer<InputComputingData>
	{
	public:
		static size_t Serialize(const InputComputingData& inData, char*& outRawData);
		static InputComputingData Deserialize(const char* inRawData);
	};


	template<>
	class FC_WEB_API WebDataSerializer<OutputComputingData>
	{
	public:
		static size_t Serialize(const OutputComputingData& inData, char*& outRawData);
		static OutputComputingData Deserialize(const char* inRawData);
	};
}
