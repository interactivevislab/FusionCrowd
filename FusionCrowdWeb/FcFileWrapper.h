#pragma once

#include "FcWebApi.h"

#include <string>


namespace FusionCrowdWeb
{
	/**
	* \class FcFileWrapper
	* \brief Wrapper class for files to transfer them in web network.
	*/
	class FC_WEB_API FcFileWrapper
	{
	public:
		FcFileWrapper() = default;
		explicit FcFileWrapper(const std::string& inFileName);

		FcFileWrapper(FcFileWrapper &&);
		FcFileWrapper & operator=(FcFileWrapper &&);

		~FcFileWrapper();

		/** Returns file size in bytes. */
		size_t GetSize() const;

		/** Changing wrapper's name for file. */
		void SetFileName(const std::string& inFileName);

		/** Return wrapper's name for file. */
		const char* GetFileName();

		/** Writes file to memory. */
		void WriteToMemory(char*& outMemoryIterator) const;

		/** Reads file from memory. */
		void ReadFromMemory(const char*& outMemoryIterator);

		/** Create file using current wrapper's content. */
		void Unwrap(const std::string& inFileName);

		/** Calculates file size in bytes. */
		static size_t GetFileSize(const std::string& inFileName);

		/** Determines full path for file in resource folder. */
		static std::string GetFullNameForResource(const std::string& inFileName);

	private:
		/** Wrapper's name for file. */
		char* _fileName = nullptr;

		/** Pointer to copy of file content. */
		char* _fileData = nullptr;

		/** Size of file name in bytes. */
		size_t _fileNameSize = 0;

		/** Size of file content in bytes. */
		size_t _fileDataSize = 0;
	};
}
