#include "FcWebData.h"

#include <cstring>


namespace FusionCrowdWeb
{
	size_t InitComputingData::Serialize(const InitComputingData& inData, char*& outRawData)
	{
		size_t agentsNum = inData.AgentsData.size();
		size_t dataSize = agentsNum * sizeof(AgentInitData) + sizeof(size_t);
		outRawData = new char[dataSize];

		auto iter = outRawData;
		WebDataHelper::WriteData(agentsNum, iter);
		for (int i = 0; i < agentsNum; i++)
		{
			WebDataHelper::WriteData(inData.AgentsData[i], iter);
		}

		return dataSize;
	}


	InitComputingData InitComputingData::Deserialize(const char* inRawData)
	{
		size_t agentsNum = WebDataHelper::ReadData<size_t>(inRawData);
		InitComputingData result{ FusionCrowd::FCArray<AgentInitData>(agentsNum) };

		for (int i = 0; i < agentsNum; i++)
		{
			result.AgentsData[i] = WebDataHelper::ReadData<AgentInitData>(inRawData);
		}

		return result;
	}


	size_t InputComputingData::Serialize(const InputComputingData& inData, char*& outRawData)
	{
		size_t dataSize = sizeof(float);
		outRawData = new char[dataSize];

		auto iter = outRawData;
		WebDataHelper::WriteData(inData.TimeStep, iter);

		return dataSize;
	}


	InputComputingData InputComputingData::Deserialize(const char* inRawData)
	{
		float timeStep = WebDataHelper::ReadData<float>(inRawData);
		return InputComputingData{ timeStep };
	}


	size_t OutputComputingData::Serialize(const OutputComputingData& inData, char*& outRawData)
	{
		size_t agentsNum = inData.AgentInfos.size();
		size_t dataSize = agentsNum * sizeof(FusionCrowd::AgentInfo) + sizeof(size_t);
		outRawData = new char[dataSize];

		auto iter = outRawData;
		WebDataHelper::WriteData(agentsNum, iter);
		for (int i = 0; i < agentsNum; i++)
		{
			WebDataHelper::WriteData(inData.AgentInfos[i], iter);
		}

		return dataSize;
	}


	OutputComputingData OutputComputingData::Deserialize(const char* inRawData)
	{
		size_t agentsNum = WebDataHelper::ReadData<size_t>(inRawData);
		OutputComputingData result{ FusionCrowd::FCArray<FusionCrowd::AgentInfo>(agentsNum) };

		for (int i = 0; i < agentsNum; i++)
		{
			result.AgentInfos[i] = WebDataHelper::ReadData<FusionCrowd::AgentInfo>(inRawData);
		}

		return result;
	}
}
