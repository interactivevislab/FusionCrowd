#pragma once

#include "Export/Export.h"


namespace FusionCrowdWeb
{
	template<typename T>
	class SimpleDataSerializer
	{
	public:
		static void Serialize(const T& inData, char*& outMemoryIterator)
		{
			size_t size = sizeof(T);
			std::memcpy(outMemoryIterator, &inData, size);
			outMemoryIterator += size;
		}


		static T Deserialize(const char*& outMemoryIterator)
		{
			T data = *reinterpret_cast<const T*>(outMemoryIterator);
			outMemoryIterator += sizeof(T);
			return data;
		}
	};


	template<typename T>
	class SimpleDataSerializer<FusionCrowd::FCArray<T>>
	{
	public:
		static void Serialize(const FusionCrowd::FCArray<T>& inData, char*& outMemoryIterator)
		{
			size_t arraySize = inData.size();
			SimpleDataSerializer<size_t>::Serialize(arraySize, outMemoryIterator);
			for (int i = 0; i < arraySize; i++)
			{
				SimpleDataSerializer<T>::Serialize(inData[i], outMemoryIterator);
			}
		}


		static FusionCrowd::FCArray<T> Deserialize(const char*& outMemoryIterator)
		{
			size_t arraySize = SimpleDataSerializer<size_t>::Deserialize(outMemoryIterator);
			auto array = FusionCrowd::FCArray<T>(arraySize);

			for (int i = 0; i < arraySize; i++)
			{
				array[i] = SimpleDataSerializer<T>::Deserialize(outMemoryIterator);
			}

			return array;
		}


		static size_t SizeOf(const FusionCrowd::FCArray<T>& inData)
		{
			return sizeof(size_t) + inData.size() * sizeof(T);
		}
	};
}
