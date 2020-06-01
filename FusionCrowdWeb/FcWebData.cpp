#include "FcWebData.h"

#include <cstring>


namespace FusionCrowdWeb
{
	void InitComputingData::Serialize(const InitComputingData& inData, char* inDest)
	{
		std::memcpy(inDest, &(inData.StubData), sizeof(float));
	}


	InitComputingData InitComputingData::Deserialize(const char* inRawData)
	{
		float data = *reinterpret_cast<const float*>(inRawData);
		InitComputingData result;
		result.StubData = data;
		return result;
	}


	void InputComputingData::Serialize(const InputComputingData& inData, char* inDest)
	{
		std::memcpy(inDest, &(inData.StubData), sizeof(float));
	}


	InputComputingData InputComputingData::Deserialize(const char* inRawData)
	{
		float data = *reinterpret_cast<const float*>(inRawData);
		InputComputingData result;
		result.StubData = data;
		return result;
	}


	void OutputComputingData::Serialize(const OutputComputingData& inData, char* inDest)
	{
		std::memcpy(inDest, &(inData.StubData), sizeof(float));
	}


	OutputComputingData OutputComputingData::Deserialize(const char* inRawData)
	{
		float data = *reinterpret_cast<const float*>(inRawData);
		OutputComputingData result;
		result.StubData = data;
		return result;
	}
}
