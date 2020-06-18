#pragma once

#include "FcWebApi.h"

#include <string>


namespace FusionCrowdWeb
{
	class FC_WEB_API FcFileWrapper
	{
	public:
		FcFileWrapper() = default;
		explicit FcFileWrapper(const std::string& inFileName);

		FcFileWrapper(FcFileWrapper &&);
		FcFileWrapper & operator=(FcFileWrapper &&);

		~FcFileWrapper();

		size_t GetSize() const;
		void SetFileName(const std::string& inFileName);
		const char* GetFileName();

		void WriteToMemory(char*& outMemoryIterator) const;
		void ReadFromMemory(const char*& outMemoryIterator);

		void Unwrap(const std::string& inFileName);

		static size_t GetFileSize(const std::string& inFileName);
		static std::string GetFullNameForResource(const std::string& inFileName);

	private:
		char* _fileName = nullptr;
		char* _fileData = nullptr;
		size_t _fileNameSize = 0;
		size_t _fileDataSize = 0;
	};
}
