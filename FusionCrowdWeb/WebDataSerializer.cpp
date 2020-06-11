#include "WebDataSerializer.h"
#include "SimpleDataSerializer.h"


namespace FusionCrowdWeb
{
	using namespace FusionCrowd;

	size_t WebDataSerializer<InitComputingData>::Serialize(const InitComputingData& inData, char*& outRawData)
	{
		size_t dataSize = inData.NavMeshFile.GetSize() 
			+ SimpleDataSerializer<FCArray<AgentInitData>>::SizeOf(inData.AgentsData) 
			+ sizeof(NavMeshRegion)
			+ SimpleDataSerializer<FCArray<AgentInitData>>::SizeOf(inData.BoundaryAgentsData);
		outRawData = new char[dataSize];
		auto iter = outRawData;

		inData.NavMeshFile.WriteToMemory(iter);
		SimpleDataSerializer<FCArray<AgentInitData>>::Serialize(inData.AgentsData, iter);
		SimpleDataSerializer<NavMeshRegion>::Serialize(inData.NavMeshRegion, iter);
		SimpleDataSerializer<FCArray<AgentInitData>>::Serialize(inData.BoundaryAgentsData, iter);

		return dataSize;
	}


	InitComputingData WebDataSerializer<InitComputingData>::Deserialize(const char* inRawData)
	{
		InitComputingData result;

		result.NavMeshFile.ReadFromMemory(inRawData);
		result.AgentsData = SimpleDataSerializer<FCArray<AgentInitData>>::Deserialize(inRawData);
		result.NavMeshRegion = SimpleDataSerializer<NavMeshRegion>::Deserialize(inRawData);
		result.BoundaryAgentsData = SimpleDataSerializer<FCArray<AgentInitData>>::Deserialize(inRawData);

		return std::move(result);
	}


	size_t WebDataSerializer<InputComputingData>::Serialize(const InputComputingData& inData, char*& outRawData)
	{
		size_t dataSize = sizeof(float)
			+ SimpleDataSerializer<FCArray<AgentInfo>>::SizeOf(inData.NewAgents)
			+ SimpleDataSerializer<FCArray<AgentInfo>>::SizeOf(inData.BoundaryAgents);
		outRawData = new char[dataSize];
		auto iter = outRawData;

		SimpleDataSerializer<float>::Serialize(inData.TimeStep, iter);
		SimpleDataSerializer<FCArray<AgentInfo>>::Serialize(inData.NewAgents, iter);
		SimpleDataSerializer<FCArray<AgentInfo>>::Serialize(inData.BoundaryAgents, iter);

		return dataSize;
	}


	InputComputingData WebDataSerializer<InputComputingData>::Deserialize(const char* inRawData)
	{
		InputComputingData result;

		result.TimeStep = SimpleDataSerializer<float>::Deserialize(inRawData);
		result.NewAgents = SimpleDataSerializer<FCArray<AgentInfo>>::Deserialize(inRawData);
		result.BoundaryAgents = SimpleDataSerializer<FCArray<AgentInfo>>::Deserialize(inRawData);

		return std::move(result);
	}


	size_t WebDataSerializer<OutputComputingData>::Serialize(const OutputComputingData& inData, char*& outRawData)
	{
		size_t dataSize = SimpleDataSerializer<FCArray<AgentInfo>>::SizeOf(inData.AgentInfos)
			+ SimpleDataSerializer<FCArray<AgentInfo>>::SizeOf(inData.MissingAgents);
		outRawData = new char[dataSize];
		auto iter = outRawData;

		SimpleDataSerializer<FCArray<AgentInfo>>::Serialize(inData.AgentInfos, iter);
		SimpleDataSerializer<FCArray<AgentInfo>>::Serialize(inData.MissingAgents, iter);

		return dataSize;
	}


	OutputComputingData WebDataSerializer<OutputComputingData>::Deserialize(const char* inRawData)
	{
		OutputComputingData result;

		result.AgentInfos = SimpleDataSerializer<FCArray<AgentInfo>>::Deserialize(inRawData);
		result.MissingAgents = SimpleDataSerializer<FCArray<AgentInfo>>::Deserialize(inRawData);

		return std::move(result);
	}
}
