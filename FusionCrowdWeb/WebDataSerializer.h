#pragma once

#include "FcWebApi.h"
#include "Export/Export.h"
#include "FcWebData.h"


namespace FusionCrowdWeb
{
	/**
	* \class SimpleDataSerializer
	* \brief Class for special web data types serialization.
	*/
	template<typename T>
	class WebDataSerializer
	{
	public:
		/**
		* \fn Serialize
		* \brief Serializes data and writes it to memory.
		*
		* @param inData		Data for serialization.
		* @param outRawData	In-memory iterator for writing data.
		*/
		static size_t Serialize(const T& inData, char*& outRawData)
		{
			static_assert(sizeof(T) == 0, "Unimplemented function");
		}


		/**
		* \fn Deserialize
		* \brief Reads data from memory and deserializes it.
		*
		* @param inRawData	In-memory iterator for reading data.
		*
		* @return Deserialized data.
		*/
		static T Deserialize(const char* inRawData)
		{
			static_assert(sizeof(T) == 0, "Unimplemented function");
		}
	};


	/** WebDataSerializer specialization for InitComputingData. */
	template<>
	class FC_WEB_API WebDataSerializer<InitComputingData>
	{
	public:
		static size_t Serialize(const InitComputingData& inData, char*& outRawData);
		static InitComputingData Deserialize(const char* inRawData);
	};


	/** WebDataSerializer specialization for InputComputingData. */
	template<>
	class FC_WEB_API WebDataSerializer<InputComputingData>
	{
	public:
		static size_t Serialize(const InputComputingData& inData, char*& outRawData);
		static InputComputingData Deserialize(const char* inRawData);
	};


	/** WebDataSerializer specialization for OutputComputingData. */
	template<>
	class FC_WEB_API WebDataSerializer<OutputComputingData>
	{
	public:
		static size_t Serialize(const OutputComputingData& inData, char*& outRawData);
		static OutputComputingData Deserialize(const char* inRawData);
	};


	/** WebDataSerializer specialization for AgentsIds. */
	template<>
	class FC_WEB_API WebDataSerializer<AgentsIds>
	{
	public:
		static size_t Serialize(const AgentsIds& inData, char*& outRawData);
		static AgentsIds Deserialize(const char* inRawData);
	};
}
