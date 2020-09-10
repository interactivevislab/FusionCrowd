#include "FcFileWrapper.h"

#include "SimpleDataSerializer.h"

#include <windows.h>
#include <fstream>


namespace FusionCrowdWeb
{
	FcFileWrapper::FcFileWrapper(const char* inFileName)
	{
		SetFileName(inFileName);

		_fileDataSize = GetFileSize(inFileName);
		_fileData = new char[_fileDataSize];

		std::ifstream file;
		file.open(inFileName, std::ifstream::binary);
		file.read(_fileData, _fileDataSize);
		file.close();
	}


	inline FcFileWrapper::FcFileWrapper(FcFileWrapper&& other)
	{
		_fileName		= other._fileName;
		_fileData		= other._fileData;
		_fileNameSize	= other._fileNameSize;
		_fileDataSize	= other._fileDataSize;

		other._fileName		= nullptr;
		other._fileData		= nullptr;
		other._fileNameSize	= 0;
		other._fileDataSize	= 0;
	}


	inline FcFileWrapper& FcFileWrapper::operator=(FcFileWrapper&& other)
	{
		if (this != &other)
		{
			_fileName		= other._fileName;
			_fileData		= other._fileData;
			_fileNameSize	= other._fileNameSize;
			_fileDataSize	= other._fileDataSize;

			other._fileName		= nullptr;
			other._fileData		= nullptr;
			other._fileNameSize	= 0;
			other._fileDataSize	= 0;
		}

		return *this;
	}


	FcFileWrapper::~FcFileWrapper()
	{
		delete[] _fileName;
		delete[] _fileData;
	}


	size_t FcFileWrapper::GetSize() const
	{
		return 2 * sizeof(size_t) + _fileNameSize + _fileDataSize;
	}


	void FcFileWrapper::SetFileName(const char* inFileName)
	{
		if (_fileName != nullptr)
		{
			delete[] _fileName;
		}
		_fileNameSize = std::strlen(inFileName) + 1;
		_fileName = new char[_fileNameSize];
		std::memcpy(_fileName, inFileName, _fileNameSize);
	}


	const char* FcFileWrapper::GetFileName()
	{
		return _fileName;
	}


	void FcFileWrapper::WriteToMemory(char*& outMemoryIterator) const
	{
		SimpleDataSerializer<size_t>::Serialize(_fileNameSize, outMemoryIterator);
		SimpleDataSerializer<size_t>::Serialize(_fileDataSize, outMemoryIterator);
		std::memcpy(outMemoryIterator, _fileName, _fileNameSize);
		outMemoryIterator += _fileNameSize;
		std::memcpy(outMemoryIterator, _fileData, _fileDataSize);
		outMemoryIterator += _fileDataSize;
	}


	void FcFileWrapper::ReadFromMemory(const char*& outMemoryIterator)
	{
		_fileNameSize = SimpleDataSerializer<size_t>::Deserialize(outMemoryIterator);
		_fileDataSize = SimpleDataSerializer<size_t>::Deserialize(outMemoryIterator);
		_fileName = new char[_fileNameSize];
		_fileData = new char[_fileDataSize];
		std::memcpy(_fileName, outMemoryIterator, _fileNameSize);
		outMemoryIterator += _fileNameSize;
		std::memcpy(_fileData, outMemoryIterator, _fileDataSize);
		outMemoryIterator += _fileDataSize;
	}


	void FcFileWrapper::Unwrap(const char* inFileName)
	{
		std::ofstream file;
		file.open(inFileName, std::ifstream::binary);
		file.write(_fileData, _fileDataSize);
		file.close();
	}


	size_t FcFileWrapper::GetFileSize(const char* inFileName)
	{
		std::ifstream file;
		file.open(inFileName, std::ios::binary | std::ios::ate);
		auto size = file.tellg();
		file.close();
		return size;
	}


	const char* FcFileWrapper::GetFullNameForResource(const char* inFileName)
	{
		char exePath[MAX_PATH];
		GetModuleFileName(NULL, exePath, MAX_PATH);
		auto pos = std::string(exePath).find_last_of("\\/");
		auto pathString = std::string(exePath).substr(0, pos + 1).append("Resources\\").append(inFileName);

		char* path = new char[pathString.size() + 1];
		strcpy_s(path, pathString.size() + 1, pathString.c_str());
		return path;
	}
}
