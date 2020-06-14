#include "WebDataSerializer.h"
#include "SimpleDataSerializer.h"


namespace FusionCrowdWeb
{
	using namespace FusionCrowd;

	size_t WebDataSerializer<InitComputingData>::Serialize(const InitComputingData& inData, char*& outRawData)
	{
		size_t dataSize = inData.NavMeshFile.GetSize()
			+ SimpleDataSerializer<FCArray<AgentInitData>>::SizeOf(inData.AgentsData)
			+ sizeof(NavMeshRegion);
		outRawData = new char[dataSize];
		auto iter = outRawData;

		inData.NavMeshFile.WriteToMemory(iter);
		SimpleDataSerializer<FCArray<AgentInitData>>::Serialize(inData.AgentsData, iter);
		SimpleDataSerializer<NavMeshRegion>::Serialize(inData.NavMeshRegion, iter);

		return dataSize;
	}


	InitComputingData WebDataSerializer<InitComputingData>::Deserialize(const char* inRawData)
	{
		InitComputingData result;

		result.NavMeshFile.ReadFromMemory(inRawData);
		result.AgentsData = SimpleDataSerializer<FCArray<AgentInitData>>::Deserialize(inRawData);
		result.NavMeshRegion = SimpleDataSerializer<NavMeshRegion>::Deserialize(inRawData);

		return std::move(result);
	}


	size_t WebDataSerializer<InputComputingData>::Serialize(const InputComputingData& inData, char*& outRawData)
	{
		size_t dataSize = sizeof(float)
			+ SimpleDataSerializer<FCArray<AgentInitData>>::SizeOf(inData.NewAgents)
			+ SimpleDataSerializer<FCArray<AgentInitData>>::SizeOf(inData.BoundaryAgents);
		outRawData = new char[dataSize];
		auto iter = outRawData;

		SimpleDataSerializer<float>::Serialize(inData.TimeStep, iter);
		SimpleDataSerializer<FCArray<AgentInitData>>::Serialize(inData.NewAgents, iter);
		SimpleDataSerializer<FCArray<AgentInitData>>::Serialize(inData.BoundaryAgents, iter);

		return dataSize;
	}


	InputComputingData WebDataSerializer<InputComputingData>::Deserialize(const char* inRawData)
	{
		InputComputingData result;

		result.TimeStep = SimpleDataSerializer<float>::Deserialize(inRawData);
		result.NewAgents = SimpleDataSerializer<FCArray<AgentInitData>>::Deserialize(inRawData);
		result.BoundaryAgents = SimpleDataSerializer<FCArray<AgentInitData>>::Deserialize(inRawData);

		return std::move(result);
	}


	size_t WebDataSerializer<OutputComputingData>::Serialize(const OutputComputingData& inData, char*& outRawData)
	{
		size_t dataSize = SimpleDataSerializer<FCArray<AgentInfo>>::SizeOf(inData.AgentInfos)
			+ SimpleDataSerializer<FCArray<AgentInfo>>::SizeOf(inData.DisplacedAgents);
		outRawData = new char[dataSize];
		auto iter = outRawData;

		SimpleDataSerializer<FCArray<AgentInfo>>::Serialize(inData.AgentInfos, iter);
		SimpleDataSerializer<FCArray<AgentInfo>>::Serialize(inData.DisplacedAgents, iter);

		return dataSize;
	}


	OutputComputingData WebDataSerializer<OutputComputingData>::Deserialize(const char* inRawData)
	{
		OutputComputingData result;

		result.AgentInfos = SimpleDataSerializer<FCArray<AgentInfo>>::Deserialize(inRawData);
		result.DisplacedAgents = SimpleDataSerializer<FCArray<AgentInfo>>::Deserialize(inRawData);

		return std::move(result);
	}
}
