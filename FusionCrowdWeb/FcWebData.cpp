#include "FcWebData.h"

#include <cstring>


namespace FusionCrowdWeb
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


	size_t WebDataSerializer<InitComputingData>::Serialize(const InitComputingData& inData, char*& outRawData)
	{
		size_t agentsNum = inData.AgentsData.size();
		size_t dataSize = agentsNum * sizeof(AgentInitData) + sizeof(size_t);
		outRawData = new char[dataSize];

		auto iter = outRawData;
		WriteData(agentsNum, iter);
		for (int i = 0; i < agentsNum; i++)
		{
			WriteData(inData.AgentsData[i], iter);
		}

		return dataSize;
	}


	InitComputingData WebDataSerializer<InitComputingData>::Deserialize(const char* inRawData)
	{
		size_t agentsNum = ReadData<size_t>(inRawData);
		InitComputingData result{ FusionCrowd::FCArray<AgentInitData>(agentsNum) };

		for (int i = 0; i < agentsNum; i++)
		{
			result.AgentsData[i] = ReadData<AgentInitData>(inRawData);
		}

		return result;
	}


	size_t WebDataSerializer<InputComputingData>::Serialize(const InputComputingData& inData, char*& outRawData)
	{
		size_t dataSize = sizeof(float);
		outRawData = new char[dataSize];

		auto iter = outRawData;
		WriteData(inData.TimeStep, iter);

		return dataSize;
	}


	InputComputingData WebDataSerializer<InputComputingData>::Deserialize(const char* inRawData)
	{
		float timeStep = ReadData<float>(inRawData);
		return InputComputingData{ timeStep };
	}


	size_t WebDataSerializer<OutputComputingData>::Serialize(const OutputComputingData& inData, char*& outRawData)
	{
		size_t agentsNum = inData.AgentInfos.size();
		size_t dataSize = agentsNum * sizeof(FusionCrowd::AgentInfo) + sizeof(size_t);
		outRawData = new char[dataSize];

		auto iter = outRawData;
		WriteData(agentsNum, iter);
		for (int i = 0; i < agentsNum; i++)
		{
			WriteData(inData.AgentInfos[i], iter);
		}

		return dataSize;
	}


	OutputComputingData WebDataSerializer<OutputComputingData>::Deserialize(const char* inRawData)
	{
		size_t agentsNum = ReadData<size_t>(inRawData);
		OutputComputingData result{ FusionCrowd::FCArray<FusionCrowd::AgentInfo>(agentsNum) };

		for (int i = 0; i < agentsNum; i++)
		{
			result.AgentInfos[i] = ReadData<FusionCrowd::AgentInfo>(inRawData);
		}

		return result;
	}
}
