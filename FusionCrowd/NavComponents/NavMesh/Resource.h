#pragma once

#include "ResourceManager.h"
#include "../../Config.h"

#include <string>

class FUSION_CROWD_API Resource
{
public:
	Resource(const std::string & fileName) :_fileName(fileName), _refCount(0) {}

	void destroy();
//protected:
	virtual ~Resource() {}
public:
	const std::string & getName() const { return _fileName; }
	int incRef() {int val = ++_refCount; return val; }
	int decRef() {int val = --_refCount; return val; }
	bool isUnreferenced() const
	{
		bool val = _refCount <= 0;
		return val;
	}
	virtual const std::string & getLabel() const = 0;
//protected:
	friend class ResourceManager;
	const std::string	_fileName;
	int	_refCount;
};

template < class Rsrc >
class ResourcePtr
{
public:
	ResourcePtr(Rsrc * rsrc = 0x0) :_data(rsrc)
	{
		if (_data) _data->incRef();
	}

	ResourcePtr(const ResourcePtr< Rsrc > & rPtr) :_data(rPtr._data)
	{
		if (_data) _data->incRef();
	}

	~ResourcePtr()
	{
		if (_data) {
			_data->decRef();
			if (_data->isUnreferenced()) {
				ResourceManager::removeResource(_data);
			}
		}
	}

	ResourcePtr< Rsrc > & operator=(const ResourcePtr< Rsrc > & ptr)
	{
		if (this != &ptr) {
			if (_data) {
				_data->decRef();
				if (_data->isUnreferenced()) ResourceManager::removeResource(_data);
			}
			_data = ptr._data;
			if (_data) _data->incRef();
		}
		return *this;
	}

	Rsrc * operator->() const { return _data; }

	bool operator==(const ResourcePtr< Rsrc > & ptr) const
	{
		return _data == ptr._data;
	}

	bool hasData() const {
		return _data != 0x0;
	}

//protected:
	Rsrc *	_data;
};

